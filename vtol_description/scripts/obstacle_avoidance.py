#!/usr/bin/env python3
"""
obstacle_avoidance.py — ROS2 node for simple edge-based obstacle detection.

Subscribes to 6 RGB camera image topics from Gazebo, runs Canny edge
detection on each frame, and determines if an obstacle is "close" based
on edge density in the central region of interest.

Publishes:
  /obstacle_avoidance/status       (std_msgs/String)  — JSON with per-camera info
  /obstacle_avoidance/halt         (std_msgs/Bool)    — True = obstacle, pause mission
  /obstacle_avoidance/cam_<name>   (sensor_msgs/Image) — annotated debug images

The debug images show:
  - Canny edges overlaid in green
  - ROI rectangle (cyan when clear, red when obstacle detected)
  - Edge density percentage and OBSTACLE/CLEAR label
  - Camera name in top-left corner

Also writes halt state to /tmp/vtol_obstacle_halt so mission.py (which uses
gz.transport, not ROS2) can poll it.

Usage:
  ros2 run vtol_description obstacle_avoidance.py
  # or standalone:
  python3 obstacle_avoidance.py
  python3 obstacle_avoidance.py --threshold 0.12 --roi 0.5
"""

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

try:
    import cv2
    import numpy as np
except ImportError:
    print(
        '[obstacle_avoidance] ERROR: opencv-python and numpy are required.\n'
        '  pip install opencv-python numpy',
        file=sys.stderr,
    )
    sys.exit(1)

HALT_FILE = '/tmp/vtol_obstacle_halt'

CAMERA_TOPICS = {
    'front': '/cam_front',
    'back':  '/cam_back',
    'left':  '/cam_left',
    'right': '/cam_right',
    'up':    '/cam_up',
    'down':  '/cam_down',
}


class ObstacleAvoidanceNode(Node):
    def __init__(self, threshold: float, roi_fraction: float):
        super().__init__('obstacle_avoidance')

        self._threshold = threshold
        self._roi_fraction = roi_fraction

        # Per-camera state
        self._cam_obstacle = {name: False for name in CAMERA_TOPICS}
        self._cam_density = {name: 0.0 for name in CAMERA_TOPICS}

        # Publishers
        self._status_pub = self.create_publisher(String, '/obstacle_avoidance/status', 10)
        self._halt_pub = self.create_publisher(Bool, '/obstacle_avoidance/halt', 10)

        # Debug image publishers (annotated views)
        self._debug_pubs = {}
        for name in CAMERA_TOPICS:
            self._debug_pubs[name] = self.create_publisher(
                Image, f'/obstacle_avoidance/cam_{name}', 10,
            )

        # Subscribers
        self._subs = {}
        for name, topic in CAMERA_TOPICS.items():
            self._subs[name] = self.create_subscription(
                Image, topic,
                lambda msg, n=name: self._on_image(n, msg),
                10,
            )

        # Periodic status publish at 5 Hz
        self._timer = self.create_timer(0.2, self._publish_status)

        # Init halt file
        self._write_halt_file(False)

        self.get_logger().info(
            f'Obstacle avoidance started — threshold={threshold:.3f}, '
            f'ROI={roi_fraction:.0%}, cameras={list(CAMERA_TOPICS.keys())}'
        )

    def _on_image(self, cam_name: str, msg: Image):
        """Process a single camera frame and publish annotated debug image."""
        # Convert ROS Image to numpy array
        h, w = msg.height, msg.width
        if msg.encoding in ('rgb8', 'bgr8'):
            channels = 3
        elif msg.encoding == 'rgba8':
            channels = 4
        else:
            channels = 3  # best-effort

        expected = h * w * channels
        if len(msg.data) < expected:
            return

        img = np.frombuffer(msg.data, dtype=np.uint8)[:expected].reshape((h, w, channels))

        # Convert to grayscale
        if channels == 4:
            gray = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
        else:
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Central region of interest
        roi_margin = (1.0 - self._roi_fraction) / 2.0
        y1 = int(h * roi_margin)
        y2 = int(h * (1.0 - roi_margin))
        x1 = int(w * roi_margin)
        x2 = int(w * (1.0 - roi_margin))

        roi = edges[y1:y2, x1:x2]
        roi_pixels = roi.size
        if roi_pixels == 0:
            return

        # Edge density = fraction of pixels that are edges
        edge_count = np.count_nonzero(roi)
        density = edge_count / roi_pixels

        self._cam_density[cam_name] = density
        obstacle = density > self._threshold
        self._cam_obstacle[cam_name] = obstacle

        # ── Build annotated debug image ──
        # Work in BGR for OpenCV drawing
        if channels == 4:
            debug = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        else:
            debug = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Overlay edges in green
        edge_mask = edges > 0
        debug[edge_mask] = (0, 255, 0)

        # Draw ROI rectangle: red if obstacle, cyan if clear
        box_color = (0, 0, 255) if obstacle else (255, 255, 0)
        cv2.rectangle(debug, (x1, y1), (x2, y2), box_color, 2)

        # Status label and density
        label = 'OBSTACLE' if obstacle else 'CLEAR'
        label_color = (0, 0, 255) if obstacle else (0, 255, 0)

        # Background bar for text readability
        cv2.rectangle(debug, (0, h - 36), (w, h), (0, 0, 0), -1)
        cv2.putText(debug, f'{label}  density={density:.3f}  thr={self._threshold:.3f}',
                    (6, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_color, 1)

        # Camera name top-left
        cv2.rectangle(debug, (0, 0), (100, 24), (0, 0, 0), -1)
        cv2.putText(debug, cam_name.upper(), (6, 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Convert back to RGB and publish
        debug_rgb = cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)
        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = h
        debug_msg.width = w
        debug_msg.encoding = 'rgb8'
        debug_msg.step = w * 3
        debug_msg.data = debug_rgb.tobytes()
        self._debug_pubs[cam_name].publish(debug_msg)

    def _publish_status(self):
        """Publish obstacle status and halt signal."""
        any_obstacle = any(self._cam_obstacle.values())

        # Build status JSON
        per_cam = {}
        for name in CAMERA_TOPICS:
            per_cam[name] = {
                'obstacle': self._cam_obstacle[name],
                'edge_density': round(self._cam_density[name], 4),
            }

        # Determine recommended action
        if not any_obstacle:
            action = 'CLEAR'
        else:
            blocked = [n for n, v in self._cam_obstacle.items() if v]
            action = 'STOP_' + '_'.join(b.upper() for b in blocked)

        status = {
            'safe': not any_obstacle,
            'action': action,
            'cameras': per_cam,
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self._status_pub.publish(status_msg)

        halt_msg = Bool()
        halt_msg.data = any_obstacle
        self._halt_pub.publish(halt_msg)

        # Write halt file for mission.py interop
        self._write_halt_file(any_obstacle)

    def _write_halt_file(self, halt: bool):
        """Write halt state to a file that mission.py can poll."""
        try:
            with open(HALT_FILE, 'w') as f:
                f.write('1' if halt else '0')
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(description='Obstacle avoidance via edge detection')
    parser.add_argument(
        '--threshold', type=float, default=0.08,
        help='Edge density threshold for obstacle detection (default: 0.08)',
    )
    parser.add_argument(
        '--roi', type=float, default=0.6,
        help='Central ROI fraction of image (default: 0.6 = middle 60%%)',
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ObstacleAvoidanceNode(threshold=args.threshold, roi_fraction=args.roi)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Clean up halt file
        try:
            import os
            os.remove(HALT_FILE)
        except OSError:
            pass


if __name__ == '__main__':
    main()
