from fastapi import FastAPI, WebSocket, Request
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn, time, json, asyncio
from typing import Dict, Any

app = FastAPI()

# Allow UI & API on same server
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # better than "*"
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

registry = {}  # In-memory drone registry
telemetry_data = {}  # Store latest telemetry data
camera_streams = {}  # Store camera stream connections
command_streams = {}  # Telemetry WebSockets, used to push commands to drones
DRONE_TIMEOUT = 15  # seconds before a drone is considered offline

@app.post("/register")
async def register(drone: dict):
    registry[drone["id"]] = {"metadata": drone, "last_seen": time.time()}
    print(f"[REGISTER] Drone {drone['id']} registered")
    return {"status": "registered"}

@app.post("/heartbeat")
async def heartbeat(data: dict):
    drone_id = data["id"]
    if drone_id in registry:
        registry[drone_id]["last_seen"] = time.time()
        print(f"[HEARTBEAT] Drone {drone_id} alive")
        return {"status": "alive"}
    return {"status": "unknown"}

@app.websocket("/stream/{drone_id}")
async def stream(websocket: WebSocket, drone_id: str):
    await websocket.accept()
    print(f"[STREAM] Drone {drone_id} connected")
    # The drone listens for commands on this same socket (listen_commands
    # in drone_client.py), so keep it around for the command endpoints.
    command_streams[drone_id] = websocket
    # Drones only POST /register once at startup; make sure a reconnect
    # after a server restart still lands in the registry.
    registry.setdefault(drone_id,
                        {"metadata": {"id": drone_id}, "last_seen": time.time()})
    try:
        while True:
            data = await websocket.receive_text()
            registry[drone_id]["last_seen"] = time.time()

            # Parse and store telemetry data
            try:
                telemetry = json.loads(data)
                telemetry["drone_id"] = drone_id
                telemetry["timestamp"] = time.time()
                telemetry_data[drone_id] = telemetry

                # Broadcast to all connected dashboard clients
                await broadcast_telemetry(telemetry)

                print(f"[DATA] Drone {drone_id}: {data}")
            except json.JSONDecodeError:
                print(f"[ERROR] Invalid JSON from drone {drone_id}: {data}")

    except Exception as e:
        print(f"[DISCONNECT] Drone {drone_id}: {e}")
    finally:
        if command_streams.get(drone_id) is websocket:
            del command_streams[drone_id]

# New endpoint for dashboard WebSocket connection
@app.websocket("/stream/all")
async def stream_all(websocket: WebSocket):
    await websocket.accept()
    print("[DASHBOARD] Dashboard connected to telemetry stream")
    dashboard_connections.add(websocket)
    
    # Send initial telemetry data
    for drone_id, telemetry in telemetry_data.items():
        try:
            await websocket.send_text(json.dumps(telemetry))
        except:
            break
    
    # Keep connection alive and handle any incoming messages
    try:
        while True:
            # Just keep the connection alive
            await asyncio.sleep(1)
    except Exception as e:
        print(f"[DASHBOARD] Dashboard disconnected: {e}")
    finally:
        dashboard_connections.discard(websocket)

@app.websocket("/camera/{drone_id}")
async def camera_stream(websocket: WebSocket, drone_id: str):
    await websocket.accept()
    print(f"[CAMERA] Drone {drone_id} connected to camera stream")
    camera_streams[drone_id] = websocket
    
    try:
        while True:
            data = await websocket.receive_text()
            frame_data = json.loads(data)
            
            # Broadcast to dashboard clients
            await broadcast_camera_frame(frame_data)
            
            print(f"[CAMERA] Received frame from {drone_id}")
    except Exception as e:
        print(f"[CAMERA] Drone {drone_id} camera disconnected: {e}")
    finally:
        if drone_id in camera_streams:
            del camera_streams[drone_id]

@app.websocket("/camera/stream/all")
async def camera_stream_all(websocket: WebSocket):
    await websocket.accept()
    print("[CAMERA] Dashboard connected to camera stream")
    camera_dashboard_connections.add(websocket)
    
    try:
        while True:
            # Just keep the connection alive
            await asyncio.sleep(1)
    except Exception as e:
        print(f"[CAMERA] Dashboard camera disconnected: {e}")
    finally:
        camera_dashboard_connections.discard(websocket)

# Store active dashboard connections for broadcasting
dashboard_connections = set()
camera_dashboard_connections = set()

async def broadcast_telemetry(telemetry: Dict[str, Any]):
    """Broadcast telemetry data to all connected dashboard clients"""
    if dashboard_connections:
        message = json.dumps(telemetry)
        disconnected = set()
        
        for websocket in dashboard_connections:
            try:
                await websocket.send_text(message)
            except:
                disconnected.add(websocket)
        
        # Remove disconnected clients
        dashboard_connections.difference_update(disconnected)

async def broadcast_camera_frame(frame_data: Dict[str, Any]):
    """Broadcast camera frame to all connected dashboard clients"""
    if camera_dashboard_connections:
        message = json.dumps(frame_data)
        disconnected = set()
        
        for websocket in camera_dashboard_connections:
            try:
                await websocket.send_text(message)
            except:
                disconnected.add(websocket)
        
        # Remove disconnected clients
        camera_dashboard_connections.difference_update(disconnected)

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    """Serve the Drone Discovery UI"""
    return HTMLResponse(content=INDEX_HTML, status_code=200)

@app.get("/api/drones")
async def get_active_drones():
    """Return currently active drones"""
    now = time.time()
    active_drones = [
        drone_id for drone_id, info in registry.items()
        if now - info["last_seen"] < DRONE_TIMEOUT
    ]
    return {"active_drones": active_drones}

@app.get("/api/drones/{drone_id}")
async def get_drone_details(drone_id: str):
    """Return detailed information about a specific drone"""
    if drone_id in registry:
        drone_info = registry[drone_id]
        return {
            "id": drone_id,
            "metadata": drone_info["metadata"],
            "last_seen": drone_info["last_seen"],
            "is_active": time.time() - drone_info["last_seen"] < DRONE_TIMEOUT,
            "telemetry": telemetry_data.get(drone_id, {})
        }
    return {"error": "Drone not found"}

@app.get("/api/drones/{drone_id}/telemetry")
async def get_drone_telemetry(drone_id: str):
    """Return telemetry data for a specific drone"""
    if drone_id in registry:
        drone_info = registry[drone_id]
        return {
            "drone_id": drone_id,
            "last_seen": drone_info["last_seen"],
            "is_active": time.time() - drone_info["last_seen"] < DRONE_TIMEOUT,
            "metadata": drone_info["metadata"],
            "telemetry": telemetry_data.get(drone_id, {})
        }
    return {"error": "Drone not found"}

# Drone Control Endpoints
@app.post("/api/drones/{drone_id}/arm")
async def arm_drone(drone_id: str):
    """Send arm command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}
    
    if drone_id in command_streams:
        try:
            command = {"command": "arm", "timestamp": time.time()}
            await command_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent arm command to {drone_id}")
            return {"status": "success", "message": f"Arm command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send arm command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

@app.post("/api/drones/{drone_id}/takeoff")
async def takeoff_drone(drone_id: str, altitude: float = 10):
    """Send takeoff command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}

    if drone_id in command_streams:
        try:
            command = {"command": "takeoff", "altitude": altitude,
                       "timestamp": time.time()}
            await command_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent takeoff command to {drone_id}")
            return {"status": "success", "message": f"Takeoff command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send takeoff command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

@app.post("/api/drones/{drone_id}/land")
async def land_drone(drone_id: str):
    """Send land command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}
    
    if drone_id in command_streams:
        try:
            command = {"command": "land", "timestamp": time.time()}
            await command_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent land command to {drone_id}")
            return {"status": "success", "message": f"Land command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send land command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

@app.post("/api/drones/{drone_id}/disarm")
async def disarm_drone(drone_id: str):
    """Send disarm command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}
    
    if drone_id in command_streams:
        try:
            command = {"command": "disarm", "timestamp": time.time()}
            await command_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent disarm command to {drone_id}")
            return {"status": "success", "message": f"Disarm command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send disarm command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

@app.post("/api/drones/{drone_id}/mode")
async def set_drone_mode(drone_id: str, mode: str):
    """Send flight mode change to drone, e.g. POST .../mode?mode=GUIDED"""
    if drone_id not in registry:
        return {"error": "Drone not found"}

    if drone_id in command_streams:
        try:
            command = {"command": "set_mode", "mode": mode,
                       "timestamp": time.time()}
            await command_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent set_mode {mode} to {drone_id}")
            return {"status": "success",
                    "message": f"Mode {mode} sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send set_mode to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

# Camera Control Endpoints
@app.post("/api/drones/{drone_id}/camera/start")
async def start_camera(drone_id: str):
    """Send start camera command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}
    
    if drone_id in camera_streams:
        try:
            command = {"type": "start_camera", "timestamp": time.time()}
            await camera_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent start camera command to {drone_id}")
            return {"status": "success", "message": f"Start camera command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send start camera command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

@app.post("/api/drones/{drone_id}/camera/photo")
async def take_photo(drone_id: str):
    """Send take photo command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}
    
    if drone_id in camera_streams:
        try:
            command = {"type": "take_photo", "timestamp": time.time()}
            await camera_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent take photo command to {drone_id}")
            return {"status": "success", "message": f"Take photo command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send take photo command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

@app.post("/api/drones/{drone_id}/camera/record/start")
async def start_recording(drone_id: str):
    """Send start recording command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}
    
    if drone_id in camera_streams:
        try:
            command = {"type": "start_recording", "timestamp": time.time()}
            await camera_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent start recording command to {drone_id}")
            return {"status": "success", "message": f"Start recording command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send start recording command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

@app.post("/api/drones/{drone_id}/camera/record/stop")
async def stop_recording(drone_id: str):
    """Send stop recording command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}
    
    if drone_id in camera_streams:
        try:
            command = {"type": "stop_recording", "timestamp": time.time()}
            await camera_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent stop recording command to {drone_id}")
            return {"status": "success", "message": f"Stop recording command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send stop recording command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

@app.post("/api/drones/{drone_id}/camera/stop")
async def stop_camera(drone_id: str):
    """Send stop camera command to drone"""
    if drone_id not in registry:
        return {"error": "Drone not found"}
    
    if drone_id in camera_streams:
        try:
            command = {"type": "stop_camera", "timestamp": time.time()}
            await camera_streams[drone_id].send_text(json.dumps(command))
            print(f"[COMMAND] Sent stop camera command to {drone_id}")
            return {"status": "success", "message": f"Stop camera command sent to {drone_id}"}
        except Exception as e:
            print(f"[ERROR] Failed to send stop camera command to {drone_id}: {e}")
            return {"error": f"Failed to send command: {str(e)}"}
    else:
        return {"error": "Drone not connected"}

# ---------- UI HTML ----------
INDEX_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>I3SDK Drone Discovery Service</title>
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet">
  <style>
    body { background: linear-gradient(135deg, #667eea, #764ba2); color: white; }
    .container { margin-top: 50px; }
    .card { background: rgba(255,255,255,0.9); color: black; }
    h1 { text-align: center; margin-bottom: 30px; font-weight: bold; }
    .drone-item { font-size: 1.2em; padding: 10px; margin: 5px; border-radius: 8px; background: #f8f9fa; }
    .status-indicator { 
      width: 10px; 
      height: 10px; 
      border-radius: 50%; 
      display: inline-block; 
      margin-right: 8px; 
    }
    .status-active { background-color: #28a745; }
    .status-inactive { background-color: #dc3545; }
  </style>
</head>
<body>
  <div class="container">
    <h1>🚁 I3AerospaceSDK Discovery Service</h1>
    <div class="card p-3">
      <h4>Available Drones</h4>
      <div id="drone-list" class="mt-3">
        <p>Loading drones...</p>
      </div>
    </div>
    
    <div class="card p-3 mt-3">
      <h4>Service Status</h4>
      <div id="service-status">
        <p>Service running on port 8000</p>
        <p>WebSocket endpoint: ws://localhost:8000/stream/all</p>
      </div>
    </div>
  </div>
  
  <script>
    async function fetchDrones() {
      try {
        const res = await fetch('/api/drones');
        const data = await res.json();
        const droneList = document.getElementById('drone-list');
        droneList.innerHTML = '';
        
        if (data.active_drones.length === 0) {
          droneList.innerHTML = '<p class="text-muted">No drones available</p>';
        } else {
          data.active_drones.forEach(drone => {
            const div = document.createElement('div');
            div.className = 'drone-item';
            div.innerHTML = `
              <span class="status-indicator status-active"></span>
              🛰️ ${drone}
            `;
            droneList.appendChild(div);
          });
        }
      } catch (err) {
        console.error("Error fetching drones:", err);
      }
    }
    
    // Fetch every 3 seconds
    setInterval(fetchDrones, 3000);
    fetchDrones();
  </script>
</body>
</html>
"""

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
