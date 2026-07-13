# drone_client.py
import requests, time, json, websocket, threading, random, base64
from pymavlink import mavutil

import os
# System gz.msgs10 bindings predate the installed protobuf; the pure-python
# parser is the only implementation that accepts them.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import cv2
import numpy as np
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage

DISCOVERY_URL = "https://c244-2c0f-eb68-6ab-d400-881b-9f67-dd73-280e.ngrok-free.app"

# Gazebo down-camera topic (iris_tracking world, iris_with_front_cam model)
DOWN_CAMERA_TOPIC = ("/world/iris_tracking/model/iris/link/down_camera_link"
                     "/sensor/down_camera/image")
CAMERA_FPS = 15
JPEG_QUALITY = 70

class DroneClient:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        # self.ws_url = f"ws://localhost:8000/stream/{self.drone_id}"
        self.ws_url = f"wss://c244-2c0f-eb68-6ab-d400-881b-9f67-dd73-280e.ngrok-free.app/stream/{self.drone_id}"
        self.camera_ws_url = f"wss://c244-2c0f-eb68-6ab-d400-881b-9f67-dd73-280e.ngrok-free.app/camera/{self.drone_id}"
        self.ws = None
        self.camera_ws = None
        # Gazebo down camera (gz-transport subscription, latest frame kept)
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frames_received = 0
        self.gz_node = GzNode()
        if not self.gz_node.subscribe(GzImage, DOWN_CAMERA_TOPIC, self._on_gz_image):
            print(f"[CAMERA] WARNING: failed to subscribe to {DOWN_CAMERA_TOPIC}")
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.mav.wait_heartbeat()
        # then poll SYS_STATUS, GLOBAL_POSITION_INT, VFR_HUD, HEARTBEAT, GPS_RAW_INT, RANGEFINDER…
        self.telemetry_data = {
            'battery_voltage': 0.0,
            'battery_level': 0,
            'gps_status': 'No Fix',
            'altitude': 0.0,
            'flight_mode': 'Unknown',
            'State': 'DISARMED',
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

# ==============================================================================================================================
#   DOWN CAMERA — Gazebo gz-transport -> WebSocket (base64 JPEG frames)
# ==============================================================================================================================
    def _on_gz_image(self, msg):
        """gz-transport callback: convert the Gazebo image to a BGR frame."""
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        except ValueError:
            return  # unexpected size; skip
        if msg.pixel_format_type == 6:  # BGR_INT8
            bgr = arr
        else:                            # default treat as RGB
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        with self.frame_lock:
            self.latest_frame = bgr
            self.frames_received += 1

    def get_frame(self):
        with self.frame_lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def connect_camera_ws(self):
        while self.camera_ws is None:
            try:
                self.camera_ws = websocket.create_connection(self.camera_ws_url)
                print(f"[CAMERA] Connected camera WebSocket for {self.drone_id}")
            except Exception as e:
                print(f"[CAMERA] Camera WS connect failed: {e}")
                time.sleep(5)

    def stream_camera(self):
        """Send down-camera frames to the server as base64 JPEG JSON messages."""
        self.connect_camera_ws()
        interval = 1.0 / CAMERA_FPS
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        frame_count = 0
        while True:
            t0 = time.time()
            frame = self.get_frame()
            if frame is None:
                time.sleep(0.1)
                continue
            ok, jpg = cv2.imencode('.jpg', frame, encode_params)
            if not ok:
                continue
            frame_data = {
                'drone_id': self.drone_id,
                'camera': 'down',
                'frame': base64.b64encode(jpg.tobytes()).decode('utf-8'),
                'timestamp': time.time(),
                'width': frame.shape[1],
                'height': frame.shape[0]
            }
            try:
                self.camera_ws.send(json.dumps(frame_data))
                frame_count += 1
                if frame_count % (CAMERA_FPS * 5) == 0:
                    print(f"[CAMERA] {self.drone_id}: sent {frame_count} frames "
                          f"(rx {self.frames_received} from gz)")
            except Exception as e:
                print(f"[CAMERA] WS send failed: {e}")
                self.camera_ws = None
                self.connect_camera_ws()
            dt = time.time() - t0
            if dt < interval:
                time.sleep(interval - dt)

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
                armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                state = "ARMED" if armed else "DISARMED"
                self.telemetry_data['State'] = state
                mode_map = self.mav.mode_mapping() or {}
                inv = {v: k for k, v in mode_map.items()}
                self.telemetry_data['flight_mode'] = inv.get(msg.custom_mode, 'Unknown')

            elif t == 'SYS_STATUS':
                self.telemetry_data['battery_voltage'] = msg.voltage_battery / 1000.0
                self.telemetry_data['battery_level'] = max(0, msg.battery_remaining)

            elif t == 'GPS_RAW_INT':
                fix = {0: 'No Fix', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix', 4: 'DGPS', 5: 'RTK Float', 6: 'RTK Fixed'}
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
#   COMMAND HANDLING — receive from server, send to MAVLink
# ==============================================================================================================================
    def arm(self):
        """Arm the vehicle via MAV_CMD_COMPONENT_ARM_DISARM."""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,            # confirmation
            1,            # param1: 1 = arm
            0, 0, 0, 0, 0, 0)
        print(f"[CMD] ARM sent")

    def disarm(self, force=False):
        """Disarm the vehicle. Set force=True to override safety checks."""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,                       # confirmation
            0,                       # param1: 0 = disarm
            21196 if force else 0,   # param2: 21196 = force disarm magic number
            0, 0, 0, 0, 0)
        print(f"[CMD] DISARM sent (force={force})")

    def set_flight_mode(self, mode_name):
        """Change the flight mode by name, e.g. 'GUIDED', 'STABILIZE', 'RTL'."""
        mode_map = self.mav.mode_mapping() or {}
        mode = mode_name.upper()
        if mode not in mode_map:
            print(f"[CMD] Unknown flight mode '{mode_name}'. "
                  f"Available: {list(mode_map.keys())}")
            return
        mode_id = mode_map[mode]
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        print(f"[CMD] SET_MODE -> {mode}")

    def handle_command(self, raw):
        """Parse a command message from the server and dispatch it to MAVLink."""
        try:
            data = json.loads(raw) if isinstance(raw, str) else raw
        except (ValueError, TypeError) as e:
            print(f"[CMD] Bad command payload: {raw!r} ({e})")
            return

        command = str(data.get("command", "")).lower()
        if command == "arm":
            self.arm()
        elif command == "disarm":
            self.disarm(force=bool(data.get("force", False)))
        elif command in ("set_mode", "mode", "change_mode"):
            mode = data.get("mode") or data.get("flight_mode")
            if mode:
                self.set_flight_mode(mode)
            else:
                print(f"[CMD] set_mode missing 'mode' field: {data}")
        else:
            print(f"[CMD] Unknown command: {data}")

    def listen_commands(self):
        """Receive command messages from the server over the WebSocket."""
        while True:
            if self.ws is None:
                time.sleep(1)
                continue
            try:
                raw = self.ws.recv()
                if not raw:
                    continue
                print(f"[RECV] {self.drone_id}: {raw}")
                self.handle_command(raw)
            except Exception as e:
                print(f"[ERROR] WS recv failed: {e}")
                self.ws = None
                time.sleep(1)

# ==============================================================================================================================

    def stream_data(self):
        self.connect_ws()
        while True:
            self.read_telemetry()
            try:
                self.ws.send(json.dumps(self.telemetry_data))
                print(f"[SENT] {self.drone_id}: {self.telemetry_data}")
            except Exception as e:
                print(f"[ERROR] WS send failed: {e}")
                self.ws = None
                self.connect_ws()
            time.sleep(3)

    def start(self):
        self.register()
        threading.Thread(target=self.heartbeat, daemon=True).start()
        self.connect_mavlink()
        threading.Thread(target=self.listen_commands, daemon=True).start()
        threading.Thread(target=self.stream_camera, daemon=True).start()
        self.stream_data()


if __name__ == "__main__":
    import sys
    drone_id = sys.argv[1] if len(sys.argv) > 1 else "drone001"
    DroneClient(drone_id).start()
