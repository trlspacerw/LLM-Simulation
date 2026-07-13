#!/usr/bin/env python3
"""
Stream the Gazebo (Harmonic) camera over a TCP socket as JPEG frames.

Designed to be exposed through an ngrok TCP tunnel:
    ngrok tcp 8554
Then point the remote viewer at the ngrok host:port.

Wire format (length-prefixed framing, robust through any TCP tunnel):
    [4 bytes big-endian uint32 = JPEG length] [N bytes JPEG payload] ...

Usage:
    # auto-discover the first camera image topic
    python3 gazebo_camera_stream.py --host 0.0.0.0 --port 8554

    # or pin a specific topic
    python3 gazebo_camera_stream.py --topic /world/iris_runway/.../image
"""

import argparse
import socket
import struct
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


# ---------- topic discovery ----------

def discover_image_topic(preferred_substr: str = "front_camera") -> str:
    """Ask `gz topic -l` for topics and pick a camera image topic."""
    out = subprocess.check_output(["gz", "topic", "-l"], text=True, timeout=5)
    topics = [t.strip() for t in out.splitlines() if t.strip()]
    image_topics = [t for t in topics if "/image" in t or t.endswith("image")]
    if not image_topics:
        raise RuntimeError("No image topics found. Is the Gazebo sim running?")
    for t in image_topics:
        if preferred_substr in t:
            return t
    return image_topics[0]


# ---------- frame source ----------

class GzCameraSource:
    """Subscribes to a Gazebo image topic and keeps the latest frame."""

    def __init__(self, topic: str):
        self.topic = topic
        self.latest_bgr: np.ndarray | None = None
        self.lock = threading.Lock()
        self.frames_received = 0
        self.node = Node()
        ok = self.node.subscribe(Image, topic, self._on_image)
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {topic}")

    def _on_image(self, msg: Image):
        # gz Image pixel_format_type: RGB_INT8 = 3, BGR_INT8 = 6 (most cameras are RGB_INT8)
        w, h = msg.width, msg.height
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        except ValueError:
            return  # unexpected size; skip
        if msg.pixel_format_type == 6:  # BGR_INT8
            bgr = arr
        else:                            # default treat as RGB
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.latest_bgr = bgr
            self.frames_received += 1

    def get(self) -> np.ndarray | None:
        with self.lock:
            return None if self.latest_bgr is None else self.latest_bgr.copy()


# ---------- TCP server ----------

def serve(source: GzCameraSource, host: str, port: int,
          fps: float, jpeg_quality: int):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[stream] listening on {host}:{port}  (expose with: ngrok tcp {port})")

    interval = 1.0 / max(fps, 1.0)
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]

    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[stream] client connected from {addr}")
        sent = 0
        t_log = time.time()
        try:
            while True:
                t0 = time.time()
                frame = source.get()
                if frame is None:
                    time.sleep(0.05)
                    continue
                ok, jpg = cv2.imencode(".jpg", frame, encode_params)
                if not ok:
                    continue
                payload = jpg.tobytes()
                try:
                    conn.sendall(struct.pack(">I", len(payload)) + payload)
                except (BrokenPipeError, ConnectionResetError):
                    print("[stream] client disconnected")
                    break
                sent += 1
                if time.time() - t_log >= 5.0:
                    print(f"[stream] tx={sent} frames "
                          f"rx={source.frames_received} from gz "
                          f"size={len(payload)/1024:.1f} KB")
                    t_log = time.time()
                # pace
                dt = time.time() - t0
                if dt < interval:
                    time.sleep(interval - dt)
        finally:
            conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0",
                    help="bind interface (0.0.0.0 for ngrok)")
    ap.add_argument("--port", type=int, default=8554)
    ap.add_argument("--topic", default=None,
                    help="gz image topic; auto-discovered if omitted")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--quality", type=int, default=70,
                    help="JPEG quality 1-100 (lower = less bandwidth)")
    args = ap.parse_args()

    topic = args.topic or discover_image_topic()
    print(f"[stream] subscribing to: {topic}")
    source = GzCameraSource(topic)

    # wait for first frame so we know it's flowing
    for _ in range(50):
        if source.get() is not None:
            break
        time.sleep(0.1)
    if source.get() is None:
        print("[stream] WARNING: no frames received yet; continuing anyway")

    try:
        serve(source, args.host, args.port, args.fps, args.quality)
    except KeyboardInterrupt:
        print("\n[stream] bye")
        sys.exit(0)


if __name__ == "__main__":
    main()
