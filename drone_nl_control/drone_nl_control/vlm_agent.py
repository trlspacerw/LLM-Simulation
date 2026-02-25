#!/usr/bin/env python3
"""
VLMAgent — ROS2 node that subscribes to the front camera and answers
binary detection queries via Ollama (gemma3:4b multimodal).
"""

import base64
import threading

import cv2
import requests
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VLMAgent(Node):
    """ROS2 node: caches the latest front-camera frame and answers VLM queries."""

    def __init__(self, ollama_base_url='http://localhost:11434',
                 vlm_model='gemma3:4b', temperature=0.2):
        super().__init__('vlm_agent')
        self._base_url = ollama_base_url.rstrip('/')
        self._model = vlm_model
        self._temperature = temperature
        self._bridge = CvBridge()
        self._frame = None
        self._frame_lock = threading.Lock()

        self._sub = self.create_subscription(
            Image, '/front_camera/image', self._image_cb, 10)

        self.get_logger().info(
            f'VLMAgent ready (model={vlm_model}), '
            f'waiting for /front_camera/image ...')

    # ── image subscriber ──────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')
            return
        with self._frame_lock:
            self._frame = frame

    # ── public API ────────────────────────────────────────────────────────

    def get_latest_frame(self):
        """Return a copy of the most recent camera frame, or None."""
        with self._frame_lock:
            return None if self._frame is None else self._frame.copy()

    def ask(self, target_description: str) -> bool:
        """Query the VLM: is *target_description* visible in the current frame?

        Returns True if detected, False otherwise.
        Blocks until a response is received from Ollama.
        """
        frame = self.get_latest_frame()
        if frame is None:
            self.get_logger().warn('VLM ask: no camera frame available yet')
            return False

        # Encode frame as JPEG base64
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            self.get_logger().error('VLM ask: JPEG encode failed')
            return False
        img_b64 = base64.b64encode(buf.tobytes()).decode('utf-8')

        prompt = (
            f'Is {target_description} visible in this image? '
            f'Answer ONLY Yes or No.'
        )
        options = {'temperature': self._temperature, 'num_predict': 5}
        # llava's CLIP vision encoder crashes on GPU (SIGSEGV in CUDA); force CPU
        if 'llava' in self._model.lower():
            options['num_gpu'] = 0
        payload = {
            'model': self._model,
            'messages': [
                {
                    'role': 'user',
                    'content': prompt,
                    'images': [img_b64],
                }
            ],
            'stream': False,
            'options': options,
        }

        try:
            resp = requests.post(
                f'{self._base_url}/api/chat',
                json=payload,
                timeout=120,
            )
            resp.raise_for_status()
            data = resp.json()
            content = data.get('message', {}).get('content', '').strip().lower()
            self.get_logger().info(
                f'VLM [{target_description}] → "{content}"')
            return content.startswith('yes')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'VLM request failed: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'VLM unexpected error: {e}')
            return False
