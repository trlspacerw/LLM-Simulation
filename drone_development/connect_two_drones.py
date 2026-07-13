"""
Connect to TWO independent simulated drones and access each drone's camera.

Drone 1: MAVLink tcp:127.0.0.1:5760, camera topic '.../iris/...'
Drone 2: MAVLink tcp:127.0.0.1:5770, camera topic '.../iris2/...'

Run after launching the sim:
    bash ~/ardu_ws/src/drone_development/launch_two_drones.sh

Then:
    python3 connect_two_drones.py
"""

import threading
import time

import cv2
import numpy as np
import pymavlink.mavutil

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


# ===========================================================================
# Generic drone client: one MAVLink + one Gazebo camera per drone
# ===========================================================================

class Drone:
    def __init__(self, name: str, mav_uri: str, cam_topic: str):
        self.name = name
        self.cam_topic = cam_topic

        # MAVLink (SITL TCP). Connect direct, no MAVProxy needed.
        print(f"[{name}] connecting MAVLink {mav_uri}")
        self.mav = pymavlink.mavutil.mavlink_connection(mav_uri)
        hb = self.mav.wait_heartbeat(timeout=15)
        if hb is None:
            raise RuntimeError(f"[{name}] no heartbeat on {mav_uri}")
        print(f"[{name}] heartbeat sys={self.mav.target_system} "
              f"comp={self.mav.target_component}")

        # Gazebo camera
        self._latest = None
        self._lock = threading.Lock()
        self._node = Node()
        if not self._node.subscribe(Image, cam_topic, self._on_image):
            raise RuntimeError(f"[{name}] failed to subscribe {cam_topic}")
        print(f"[{name}] camera subscribed: {cam_topic}")

    def _on_image(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return
        bgr = arr if msg.pixel_format_type == 6 else cv2.cvtColor(
            arr, cv2.COLOR_RGB2BGR)
        with self._lock:
            self._latest = bgr

    def get_frame(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def poll_telemetry(self):
        """Return (alt_m, lat, lon) or None."""
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg is None:
            return None
        return (msg.relative_alt / 1000.0, msg.lat / 1e7, msg.lon / 1e7)


# ===========================================================================
# Main: two side-by-side feeds, independent MAVLink streams
# ===========================================================================

WORLD = "iris_tracking_two"

drone1 = Drone(
    name="drone1",
    mav_uri="tcp:127.0.0.1:5760",
    cam_topic=f"/world/{WORLD}/model/iris/link/"
              f"front_camera_link/sensor/front_camera/image",
)

drone2 = Drone(
    name="drone2",
    mav_uri="tcp:127.0.0.1:5770",
    cam_topic=f"/world/{WORLD}/model/iris2/link/"
              f"front_camera_link/sensor/front_camera/image",
)

# wait for first frames
for d in (drone1, drone2):
    for _ in range(50):
        if d.get_frame() is not None:
            break
        time.sleep(0.1)

print("[main] running. press 'q' in either window to quit.")
last_log = 0.0
while True:
    f1 = drone1.get_frame()
    f2 = drone2.get_frame()
    if f1 is not None:
        cv2.imshow("drone1", f1)
    if f2 is not None:
        cv2.imshow("drone2", f2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    t1 = drone1.poll_telemetry()
    t2 = drone2.poll_telemetry()
    if time.time() - last_log > 1.0:
        if t1:
            print(f"[drone1] alt={t1[0]:.2f} m  lat={t1[1]:.6f}  lon={t1[2]:.6f}")
        if t2:
            print(f"[drone2] alt={t2[0]:.2f} m  lat={t2[1]:.6f}  lon={t2[2]:.6f}")
        last_log = time.time()

cv2.destroyAllWindows()
