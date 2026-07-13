"""
Simple drone telemetry + camera uploader.

Reads from MAVLink (SITL or real vehicle):
    - battery voltage (V)
    - battery current (A)
    - power (V * A, computed)
    - GPS lat/lon
    - altitude (relative, m)
    - current flight mode

Reads from Gazebo camera topic:
    - latest BGR frame -> JPEG -> base64

Streams everything as JSON over a WebSocket every N seconds.

Requirements:
    pip install pymavlink opencv-python numpy websocket-client
    sudo apt install python3-gz-transport13 python3-gz-msgs10
"""

import base64
import json
import threading
import time

import cv2
import numpy as np
import websocket
from pymavlink import mavutil

try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    HAVE_GZ = True
except ImportError:
    HAVE_GZ = False


# --- config ---------------------------------------------------------------

MAVLINK_URL  = "udpin:127.0.0.1:14550"
CAMERA_TOPIC = ("/world/iris_tracking/model/iris/link/"
                "front_camera_link/sensor/front_camera/image")
SERVER_WS_URL = "ws://localhost:8000/telemetry"    # change to your endpoint
SEND_PERIOD   = 1.0   # seconds between uploads
JPEG_QUALITY  = 70
RECONNECT_DELAY = 3.0  # seconds to wait before reconnecting


# --- camera ---------------------------------------------------------------

class GzCamera:
    def __init__(self, topic):
        self._latest = None
        self._lock = threading.Lock()
        self._node = Node()
        if not self._node.subscribe(Image, topic, self._on_image):
            raise RuntimeError(f"failed to subscribe to {topic}")

    def _on_image(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return
        bgr = arr if msg.pixel_format_type == 6 else cv2.cvtColor(
            arr, cv2.COLOR_RGB2BGR)
        with self._lock:
            self._latest = bgr

    def get_jpeg_b64(self):
        with self._lock:
            frame = None if self._latest is None else self._latest.copy()
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return None
        return base64.b64encode(buf.tobytes()).decode("ascii")


# --- mavlink state --------------------------------------------------------

class Telemetry:
    def __init__(self, url):
        print(f"[mav] connecting to {url} ...")
        self.conn = mavutil.mavlink_connection(url)
        self.conn.wait_heartbeat(timeout=10)
        print(f"[mav] heartbeat from sys={self.conn.target_system} "
              f"comp={self.conn.target_component}")

        self.voltage = 0.0    # volts
        self.current = 0.0    # amps
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0        # meters, relative
        self.mode = "UNKNOWN"

    def pump(self):
        """Drain any pending MAVLink messages into the state fields."""
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "SYS_STATUS":
                # voltage_battery: mV, current_battery: cA (-1 if unknown)
                self.voltage = msg.voltage_battery / 1000.0
                self.current = (msg.current_battery / 100.0
                                if msg.current_battery != -1 else 0.0)
            elif t == "GLOBAL_POSITION_INT":
                self.lat = msg.lat / 1e7
                self.lon = msg.lon / 1e7
                self.alt = msg.relative_alt / 1000.0
            elif t == "HEARTBEAT":
                self.mode = mavutil.mode_string_v10(msg)

    def snapshot(self):
        return {
            "voltage": round(self.voltage, 3),
            "current": round(self.current, 3),
            "power":   round(self.voltage * self.current, 3),
            "latitude":  self.lat,
            "longitude": self.lon,
            "altitude":  round(self.alt, 2),
            "flight_mode": self.mode,
            "timestamp": time.time(),
        }


# --- main loop ------------------------------------------------------------

def connect_ws():
    """Block until a WebSocket connection is established."""
    while True:
        try:
            ws = websocket.create_connection(SERVER_WS_URL, timeout=5)
            print(f"[ws] connected to {SERVER_WS_URL}")
            return ws
        except Exception as e:
            print(f"[ws] connect failed: {e} — retrying in {RECONNECT_DELAY}s")
            time.sleep(RECONNECT_DELAY)


def main():
    tel = Telemetry(MAVLINK_URL)
    cam = GzCamera(CAMERA_TOPIC) if HAVE_GZ else None
    if cam is None:
        print("[camera] gz-transport not available, sending telemetry only")

    print(f"[server] streaming to {SERVER_WS_URL} every {SEND_PERIOD}s "
          "(Ctrl+C to stop)")
    ws = connect_ws()

    while True:
        loop_start = time.time()
        tel.pump()
        payload = tel.snapshot()
        if cam is not None:
            payload["camera_jpeg_b64"] = cam.get_jpeg_b64()

        try:
            ws.send(json.dumps(payload))
            print(f"[send] ok  mode={payload['flight_mode']}  "
                  f"V={payload['voltage']:.2f} I={payload['current']:.2f} "
                  f"P={payload['power']:.2f}  alt={payload['altitude']:.1f}m")
        except (websocket.WebSocketException, OSError) as e:
            print(f"[send] FAILED: {e} — reconnecting")
            try:
                ws.close()
            except Exception:
                pass
            ws = connect_ws()
            continue

        sleep_for = SEND_PERIOD - (time.time() - loop_start)
        if sleep_for > 0:
            time.sleep(sleep_for)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[exit] stopped by user")
