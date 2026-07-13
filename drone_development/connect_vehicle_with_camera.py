"""
Connect to the simulated drone (MAVLink) AND access the Gazebo front camera
in the same process. No ngrok, no sockets — direct gz-transport subscription.

Requires (already installed on this machine):
    sudo apt install python3-gz-transport13 python3-gz-msgs10
    pip install pymavlink opencv-python numpy
"""

import threading
import time

import cv2
import numpy as np
import pymavlink.mavutil

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


# --- camera ---------------------------------------------------------------

class GzCamera:
    """Subscribes to a Gazebo camera image topic and holds the latest BGR frame."""

    def __init__(self,
                 topic: str = "/world/iris_tracking/model/iris/link/"
                              "front_camera_link/sensor/front_camera/image"):
        self._latest = None
        self._lock = threading.Lock()
        self._node = Node()
        if not self._node.subscribe(Image, topic, self._on_image):
            raise RuntimeError(f"failed to subscribe to {topic}")
        print(f"[camera] subscribed to {topic}")

    def _on_image(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return
        # pixel_format_type: 3 = RGB_INT8, 6 = BGR_INT8
        bgr = arr if msg.pixel_format_type == 6 else cv2.cvtColor(
            arr, cv2.COLOR_RGB2BGR)
        with self._lock:
            self._latest = bgr

    def get_frame(self):
        """Return the most recent frame as a BGR numpy array, or None."""
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def wait_for_first_frame(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get_frame() is not None:
                return True
            time.sleep(0.05)
        return False


# --- main -----------------------------------------------------------------

# connect to the vehicle
vehicle = pymavlink.mavutil.mavlink_connection(device='udpin:127.0.0.1:14550')

# wait for the heartbeat message to find the system ID
vehicle.wait_heartbeat(timeout=5)

# debugging message
print("connected to the vehicle")
print("Target system ID:", vehicle.target_system,
      "Target component ID:", vehicle.target_component)

# open the camera
camera = GzCamera()
if not camera.wait_for_first_frame(timeout=5.0):
    print("[camera] WARNING: no frames yet — is gz sim running?")

# example usage: show live feed + print altitude from MAVLink
# press 'q' in the window to quit
print("[main] showing live feed. press 'q' to quit.")
last_print = 0.0
while True:
    frame = camera.get_frame()
    if frame is not None:
        cv2.imshow("drone camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg and time.time() - last_print > 1.0:
        print(f"[mav] alt={msg.relative_alt/1000:.2f} m  "
              f"lat={msg.lat/1e7:.6f} lon={msg.lon/1e7:.6f}")
        last_print = time.time()

cv2.destroyAllWindows()
