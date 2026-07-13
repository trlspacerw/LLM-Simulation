import requests, time, json, websocket, threading, random, base64, io
from PIL import Image, ImageDraw, ImageFont
import cv2
import numpy as np
from pymavlink import mavutil

DISCOVERY_URL = "http://localhost:8001"
# DISCOVERY_URL="https://966503904055.ngrok-free.app"

class DroneClient:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.ws_url = f"ws://localhost:8001/stream/{self.drone_id}"
        self.camera_ws_url = f"ws://localhost:8001/camera/{self.drone_id}"
        self.ws = None
        self.camera_ws = None
        self.mav = None
        self.camera_active = False
        self.recording = False
        self.armed = False
        self.flying = False

        self.telemetry_data = {
            'battery_voltage': 0.0,
            'battery_level': 0,
            'gps_status': 'No Fix',
            'altitude': 0.0,
            'flight_mode': 'Unknown',
            'armed': False,
            'rangefinder_distance': 0.0,
            'rangefinder_available': False,
            'gps_satellites': 0,
            'groundspeed': 0.0,
            'airspeed': 0.0
        }

    def register(self):
        payload = {
            "id": self.drone_id,
            "type": "quadcopter",
            "capabilities": ["gps", "battery", "altitude", "mode", "camera"]
        }
        try:
            r = requests.post(f"{DISCOVERY_URL}/register", json=payload)
            print(f"[REGISTERED] {self.drone_id} -> {r.json()}")
        except Exception as e:
            print(f"[ERROR] Register failed: {e}")

    def heartbeat(self):
        while True:
            try:
                requests.post(f"{DISCOVERY_URL}/heartbeat", json={"id": self.drone_id})
            except Exception as e:
                print(f"[ERROR] Heartbeat failed: {e}")
            time.sleep(10)

    def connect_ws(self):
        while self.ws is None:
            try:
                self.ws = websocket.create_connection(self.ws_url)
                print(f"[DISCOVERY] Connected WebSocket for {self.drone_id}")
            except Exception as e:
                print(f"[DISCOVERY] WS connect failed: {e}")
                time.sleep(5)

    def connect_camera_ws(self):
        while self.camera_ws is None and self.camera_active:
            try:
                self.camera_ws = websocket.create_connection(self.camera_ws_url)
                print(f"[CAMERA] Connected camera WebSocket for {self.drone_id}")
                
                # Start listening for commands in a separate thread
                threading.Thread(target=self.listen_for_commands, daemon=True).start()
            except Exception as e:
                print(f"[CAMERA] Camera WS connect failed: {e}")
                time.sleep(5)

    def initialize_camera(self):
        """Initialize the PC camera"""
        try:
            # Check if camera is already initialized
            if hasattr(self, 'cap') and self.cap.isOpened():
                return True
                
            # Try to open the default camera (usually index 0)
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                # If default camera fails, try index 1
                self.cap = cv2.VideoCapture(1)
            
            if self.cap.isOpened():
                # Set camera properties for better performance
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                # Add a small delay to let camera stabilize
                time.sleep(0.5)
                print(f"[CAMERA] PC camera initialized successfully for {self.drone_id}")
                return True
            else:
                print(f"[CAMERA] Failed to initialize PC camera for {self.drone_id}")
                return False
        except Exception as e:
            print(f"[CAMERA] Error initializing camera: {e}")
            return False

    def capture_camera_frame(self):
        """Capture a frame from the PC camera with telemetry overlay"""
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            return None
        
        try:
            # Capture frame from camera
            ret, frame = self.cap.read()
            if not ret:
                return None
            
            # Resize frame to standard size
            frame = cv2.resize(frame, (640, 480))
            
            # Add telemetry overlay
            overlay = frame.copy()
            cv2.rectangle(overlay, (10, 10), (300, 120), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
            
            # Add text overlay
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, f"DRONE: {self.drone_id}", (20, 35), font, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"ALT: {self.telemetry_data.get('altitude', 0):.1f}m", (20, 60), font, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"SPD: {self.telemetry_data.get('groundspeed', 0):.1f}km/h", (20, 80), font, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"BAT: {self.telemetry_data.get('battery_level', 0)}%", (20, 100), font, 0.5, (255, 255, 255), 1)
            
            # Add timestamp
            timestamp = time.strftime("%H:%M:%S")
            cv2.putText(frame, timestamp, (500, 30), font, 0.6, (255, 255, 255), 2)
            
            # Add recording indicator
            if self.camera_active:
                cv2.circle(frame, (600, 30), 8, (0, 0, 255), -1)
                cv2.putText(frame, "REC", (580, 50), font, 0.4, (255, 255, 255), 1)
            
            return frame
            
        except Exception as e:
            print(f"[CAMERA] Error capturing frame: {e}")
            return None

    def stream_camera(self):
        """Stream camera frames via WebSocket"""
        # Initialize camera first
        if not self.initialize_camera():
            print(f"[CAMERA] Failed to initialize camera for {self.drone_id}")
            return
        
        self.connect_camera_ws()
        
        frame_count = 0
        while self.camera_active and self.camera_ws:
            try:
                # Capture frame from PC camera
                frame = self.capture_camera_frame()
                
                if frame is not None:
                    # Encode frame as JPEG
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    frame_bytes = buffer.tobytes()
                    
                    # Convert to base64
                    frame_b64 = base64.b64encode(frame_bytes).decode('utf-8')
                    
                    # Send frame data
                    frame_data = {
                        'drone_id': self.drone_id,
                        'frame': frame_b64,
                        'timestamp': time.time(),
                        'width': 640,
                        'height': 480
                    }
                    
                    self.camera_ws.send(json.dumps(frame_data))
                    frame_count += 1
                    
                    # Log every 30 frames (once per second at 30fps)
                    if frame_count % 30 == 0:
                        print(f"[CAMERA] Streaming frame {frame_count} for {self.drone_id}")
                else:
                    print(f"[CAMERA] Failed to capture frame for {self.drone_id}")
                
                # Control frame rate (30 FPS)
                time.sleep(1/30)
                
            except Exception as e:
                print(f"[CAMERA] Stream error: {e}")
                self.camera_ws = None
                if self.camera_active:
                    self.connect_camera_ws()
                else:
                    break

    def start_camera(self):
        """Start camera streaming"""
        if not self.camera_active:
            self.camera_active = True
            threading.Thread(target=self.stream_camera, daemon=True).start()
            print(f"[CAMERA] Started camera streaming for {self.drone_id}")

    def stop_camera(self):
        """Stop camera streaming"""
        self.camera_active = False
        if self.camera_ws:
            self.camera_ws.close()
            self.camera_ws = None
        
        # Release camera resources
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            print(f"[CAMERA] Released camera resources for {self.drone_id}")
        
        print(f"[CAMERA] Stopped camera streaming for {self.drone_id}")

    def listen_for_commands(self):
        """Listen for commands from the dashboard"""
        while self.camera_ws and self.camera_active:
            try:
                # Check for incoming messages
                message = self.camera_ws.recv()
                if message:
                    command = json.loads(message)
                    self.execute_command(command)
            except Exception as e:
                print(f"[COMMAND] Error receiving command: {e}")
                break

    def execute_command(self, command):
        """Execute a command received from the dashboard"""
        command_type = command.get('type')
        print(f"[COMMAND] Received {command_type} command for {self.drone_id}")
        
        if command_type == 'arm':
            self.arm_drone()
        elif command_type == 'takeoff':
            self.takeoff_drone()
        elif command_type == 'land':
            self.land_drone()
        elif command_type == 'disarm':
            self.disarm_drone()
        elif command_type == 'start_camera':
            self.start_camera_command()
        elif command_type == 'take_photo':
            self.take_photo()
        elif command_type == 'start_recording':
            self.start_recording()
        elif command_type == 'stop_recording':
            self.stop_recording()
        elif command_type == 'stop_camera':
            self.stop_camera_command()
        else:
            print(f"[COMMAND] Unknown command type: {command_type}")

    def arm_drone(self):
        """Arm the drone"""
        if not self.armed:
            self.armed = True
            self.telemetry_data['armed'] = True
            print(f"[DRONE] {self.drone_id} ARMED")
        else:
            print(f"[DRONE] {self.drone_id} already armed")

    def takeoff_drone(self):
        """Take off the drone"""
        if self.armed and not self.flying:
            self.flying = True
            self.telemetry_data['flight_mode'] = 'Takeoff'
            print(f"[DRONE] {self.drone_id} TAKING OFF")
            # Simulate takeoff sequence
            time.sleep(2)
            self.telemetry_data['flight_mode'] = 'Loiter'
        else:
            print(f"[DRONE] {self.drone_id} cannot takeoff - not armed or already flying")

    def land_drone(self):
        """Land the drone"""
        if self.flying:
            self.flying = False
            self.telemetry_data['flight_mode'] = 'Landing'
            print(f"[DRONE] {self.drone_id} LANDING")
            # Simulate landing sequence
            time.sleep(3)
            self.telemetry_data['flight_mode'] = 'Unknown'
        else:
            print(f"[DRONE] {self.drone_id} not flying")

    def disarm_drone(self):
        """Disarm the drone"""
        if self.armed:
            self.armed = False
            self.flying = False
            self.telemetry_data['armed'] = False
            self.telemetry_data['flight_mode'] = 'Unknown'
            print(f"[DRONE] {self.drone_id} DISARMED")
        else:
            print(f"[DRONE] {self.drone_id} not armed")

    def start_camera_command(self):
        """Start camera on command"""
        if not self.camera_active:
            self.start_camera()
            print(f"[CAMERA] {self.drone_id} camera started by command")
        else:
            print(f"[CAMERA] {self.drone_id} camera already active")

    def take_photo(self):
        """Take a photo"""
        if self.camera_active:
            # Simulate taking a photo
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            print(f"[CAMERA] {self.drone_id} taking photo at {timestamp}")
            # In a real implementation, you would save the photo here
        else:
            print(f"[CAMERA] {self.drone_id} camera not active")

    def start_recording(self):
        """Start recording video"""
        if self.camera_active and not self.recording:
            self.recording = True
            print(f"[CAMERA] {self.drone_id} started recording")
        else:
            print(f"[CAMERA] {self.drone_id} camera not active or already recording")

    def stop_recording(self):
        """Stop recording video"""
        if self.recording:
            self.recording = False
            print(f"[CAMERA] {self.drone_id} stopped recording")
        else:
            print(f"[CAMERA] {self.drone_id} not recording")

    def stop_camera_command(self):
        """Stop camera on command"""
        if self.camera_active:
            self.stop_camera()
            print(f"[CAMERA] {self.drone_id} camera stopped by command")
        else:
            print(f"[CAMERA] {self.drone_id} camera not active")

    def simulate_telemetry(self):
        """Generate simulated telemetry"""
        # Simulate GPS coordinates around New York City with realistic movement
        base_lat = 40.7128
        base_lng = -74.0060
        
        # Create circular flight pattern for better visibility on map
        time_factor = time.time() * 0.05  # Slow movement
        radius = 0.003  # Small radius for visible movement
        lat_offset = radius * np.sin(time_factor) + (random.random() - 0.5) * 0.001
        lng_offset = radius * np.cos(time_factor) + (random.random() - 0.5) * 0.001
        
        # Update telemetry based on actual drone state
        self.telemetry_data.update({
            'battery_voltage': 11.5 + random.random() * 2,
            'battery_level': max(10, random.randint(20, 100)),
            'gps_status': '3D Fix' if random.random() > 0.1 else 'No Fix',
            'altitude': random.uniform(0, 120) if self.flying else 0,
            'flight_mode': self.telemetry_data.get('flight_mode', 'Unknown'),
            'armed': self.armed,
            'rangefinder_distance': random.uniform(0.5, 10.0) if self.flying else 0,
            'rangefinder_available': self.flying and random.random() > 0.2,
            'gps_satellites': random.randint(4, 12),
            'groundspeed': random.uniform(0, 15) if self.flying else 0,
            'airspeed': random.uniform(0, 20) if self.flying else 0,
            'latitude': base_lat + lat_offset,
            'longitude': base_lng + lng_offset,
            'heading': random.uniform(0, 360),
            'vertical_speed': random.uniform(-2, 2) if self.flying else 0,
            'throttle': random.uniform(0, 100) if self.flying else 0,
            'temperature': random.uniform(15, 35),
            'humidity': random.uniform(30, 80),
            'wind_speed': random.uniform(0, 10),
            'wind_direction': random.uniform(0, 360)
        })
# ==============================================================================================================================

    def connect_mavlink(self):
        # Use UDP 14551 (the GCS port your SITL exposes per CLAUDE.md)
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        print(f"[MAVLINK] Waiting for heartbeat...")
        self.mav.wait_heartbeat()
        print(f"[MAVLINK] Connected: sys={self.mav.target_system}")

    def read_telemetry(self):
        """Drain pending MAVLink msgs and update telemetry_data."""
        # Non-blocking drain — read all messages waiting in the buffer
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()

            if t == 'HEARTBEAT':
                self.telemetry_data['armed'] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode_map = self.mav.mode_mapping() or {}
                inv = {v: k for k, v in mode_map.items()}
                self.telemetry_data['flight_mode'] = inv.get(msg.custom_mode, 'Unknown')

            elif t == 'SYS_STATUS':
                self.telemetry_data['battery_voltage'] = msg.voltage_battery / 1000.0
                self.telemetry_data['battery_level'] = max(0, msg.battery_remaining)

            elif t == 'GPS_RAW_INT':
                fix = {0: 'No Fix', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix'}
                self.telemetry_data['gps_status'] = fix.get(msg.fix_type, 'No Fix')
                self.telemetry_data['gps_satellites'] = msg.satellites_visible

            elif t == 'GLOBAL_POSITION_INT':
                self.telemetry_data['latitude'] = msg.lat / 1e7
                self.telemetry_data['longitude'] = msg.lon / 1e7
                self.telemetry_data['altitude'] = msg.relative_alt / 1000.0

            elif t == 'VFR_HUD':
                self.telemetry_data['groundspeed'] = msg.groundspeed
                self.telemetry_data['airspeed'] = msg.airspeed

            elif t in ('RANGEFINDER', 'DISTANCE_SENSOR'):
                dist = msg.distance if t == 'RANGEFINDER' else msg.current_distance / 100.0
                self.telemetry_data['rangefinder_distance'] = dist
                self.telemetry_data['rangefinder_available'] = True
# ==============================================================================================================================
    def stream_data(self):
        self.connect_ws()
        self.connect_mavlink()
        while True:
            self.read_telemetry()
            try:
                # Add drone_id to the telemetry data
                telemetry_with_id = {**self.telemetry_data, 'drone_id': self.drone_id}
                self.ws.send(json.dumps(telemetry_with_id))
                print(f"[SENT] {self.drone_id}: Battery {self.telemetry_data['battery_level']}%, Alt {self.telemetry_data['altitude']:.1f}m, GPS: {self.telemetry_data['gps_status']}")
            except Exception as e:
                print(f"[ERROR] WS send failed: {e}")
                self.ws = None
                self.connect_ws()
            time.sleep(3)

    def cleanup(self):
        """Clean up resources when shutting down"""
        self.stop_camera()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

    def start(self):
        print(f"[STARTING] Drone {self.drone_id} client...")
        self.register()
        threading.Thread(target=self.heartbeat, daemon=True).start()
        
        # Start camera streaming immediately and keep it running
        def start_camera_immediately():
            print(f"[CAMERA] Starting camera immediately for {self.drone_id}")
            self.start_camera()
        
        # Start camera after a short delay to let other connections establish
        threading.Timer(3.0, start_camera_immediately).start()
        
        # Set up cleanup on exit
        import atexit
        atexit.register(self.cleanup)
        
        self.stream_data()


if __name__ == "__main__":
    import sys
    drone_id = sys.argv[1] if len(sys.argv) > 1 else "drone001"
    print(f"Starting drone client for {drone_id}")
    print("Make sure the service_discovery.py is running on port 8001")
    print("Press Ctrl+C to stop")
    
    try:
        DroneClient(drone_id).start()
    except KeyboardInterrupt:
        print(f"\n[STOPPED] Drone {drone_id} client stopped by user")
    except Exception as e:
        print(f"[ERROR] Drone client failed: {e}")