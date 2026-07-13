"""Show both drones' Gazebo camera streams. Press 'q' to quit."""
import cv2, numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

frames = {}

def make_cb(name):
    def cb(msg):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        frames[name] = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return cb

W = "iris_tracking_two"
node = Node()
node.subscribe(Image, f"/world/{W}/model/iris/link/front_camera_link/sensor/front_camera/image",  make_cb("drone1"))
node.subscribe(Image, f"/world/{W}/model/iris2/link/front_camera_link/sensor/front_camera/image", make_cb("drone2"))

while True:
    for name, f in list(frames.items()):
        cv2.imshow(name, f)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
