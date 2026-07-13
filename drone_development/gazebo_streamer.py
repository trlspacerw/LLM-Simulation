#!/usr/bin/env python3
"""
Unified Gazebo camera + ArduPilot MAVLink streamer.

One process, two TCP servers:

  --cam-port  (default 8554)  Gazebo camera frames, JPEG framed:
                              [4-byte BE uint32 length][JPEG bytes] ...
                              Read with remote_camera_viewer.py

  --mav-port  (default 5762)  Transparent TCP relay to ArduPilot SITL
                              on 127.0.0.1:5760. Remote pymavlink clients
                              connect here exactly as if they were talking
                              to SITL directly.

Expose with two ngrok tunnels:
    ngrok tcp 8554        # camera
    ngrok tcp 5762        # mavlink

Remote camera viewer:
    python3 remote_camera_viewer.py --host <cam-ngrok-host> --port <port>

Remote MAVLink client (pymavlink):
    mavutil.mavlink_connection('tcp:<mav-ngrok-host>:<port>')
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


# ===========================================================================
# Gazebo camera source
# ===========================================================================

def discover_image_topic(preferred_substr: str = "front_camera") -> str:
    out = subprocess.check_output(["gz", "topic", "-l"], text=True, timeout=5)
    topics = [t.strip() for t in out.splitlines() if t.strip()]
    image_topics = [t for t in topics if "/image" in t]
    if not image_topics:
        raise RuntimeError("No image topics found. Is the Gazebo sim running?")
    for t in image_topics:
        if preferred_substr in t:
            return t
    return image_topics[0]


class GzCameraSource:
    """Subscribes to a Gazebo image topic and keeps the latest frame."""

    def __init__(self, topic: str):
        self.topic = topic
        self.latest_bgr: np.ndarray | None = None
        self.lock = threading.Lock()
        self.frames_received = 0
        self.node = Node()
        if not self.node.subscribe(Image, topic, self._on_image):
            raise RuntimeError(f"Failed to subscribe to {topic}")

    def _on_image(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return
        bgr = arr if msg.pixel_format_type == 6 else cv2.cvtColor(
            arr, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.latest_bgr = bgr
            self.frames_received += 1

    def get(self) -> np.ndarray | None:
        with self.lock:
            return None if self.latest_bgr is None else self.latest_bgr.copy()


# ===========================================================================
# Camera TCP server
# ===========================================================================

def serve_camera(source: GzCameraSource, host: str, port: int,
                 fps: float, jpeg_quality: int, stop_event: threading.Event):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(1.0)
    print(f"[cam ] listening on {host}:{port}  (ngrok tcp {port})", flush=True)

    interval = 1.0 / max(fps, 1.0)
    enc = [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]

    while not stop_event.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[cam ] client connected from {addr}", flush=True)
        sent = 0
        t_log = time.time()
        try:
            while not stop_event.is_set():
                t0 = time.time()
                frame = source.get()
                if frame is None:
                    time.sleep(0.05)
                    continue
                ok, jpg = cv2.imencode(".jpg", frame, enc)
                if not ok:
                    continue
                payload = jpg.tobytes()
                try:
                    conn.sendall(struct.pack(">I", len(payload)) + payload)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    print("[cam ] client disconnected", flush=True)
                    break
                sent += 1
                if time.time() - t_log >= 5.0:
                    print(f"[cam ] tx={sent} rx={source.frames_received} "
                          f"size={len(payload)/1024:.1f} KB", flush=True)
                    t_log = time.time()
                dt = time.time() - t0
                if dt < interval:
                    time.sleep(interval - dt)
        finally:
            try: conn.close()
            except OSError: pass
    srv.close()


# ===========================================================================
# MAVLink TCP relay (remote <-> SITL)
# ===========================================================================

def _pipe(src: socket.socket, dst: socket.socket, tag: str, counter: dict):
    try:
        while True:
            data = src.recv(4096)
            if not data:
                break
            dst.sendall(data)
            counter[tag] += len(data)
    except (OSError, ConnectionResetError):
        pass
    finally:
        try: src.shutdown(socket.SHUT_RD)
        except OSError: pass
        try: dst.shutdown(socket.SHUT_WR)
        except OSError: pass


def serve_mavlink(host: str, port: int,
                  sitl_host: str, sitl_port: int,
                  stop_event: threading.Event):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(1.0)
    print(f"[mav ] listening on {host}:{port}  "
          f"(relay -> {sitl_host}:{sitl_port}) (ngrok tcp {port})", flush=True)

    while not stop_event.is_set():
        try:
            client, addr = srv.accept()
        except socket.timeout:
            continue
        print(f"[mav ] client connected from {addr}", flush=True)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            sitl = socket.create_connection((sitl_host, sitl_port), timeout=5)
        except OSError as e:
            print(f"[mav ] cannot reach SITL {sitl_host}:{sitl_port}: {e}",
                  flush=True)
            client.close()
            continue
        sitl.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        counter = {"up": 0, "dn": 0}
        t_up = threading.Thread(target=_pipe,
                                args=(client, sitl, "up", counter),
                                daemon=True)
        t_dn = threading.Thread(target=_pipe,
                                args=(sitl, client, "dn", counter),
                                daemon=True)
        t_up.start(); t_dn.start()

        # status loop until either pipe closes
        t_log = time.time()
        while t_up.is_alive() and t_dn.is_alive() and not stop_event.is_set():
            time.sleep(0.5)
            if time.time() - t_log >= 5.0:
                print(f"[mav ] up={counter['up']}B dn={counter['dn']}B",
                      flush=True)
                t_log = time.time()
        for s in (client, sitl):
            try: s.close()
            except OSError: pass
        print("[mav ] session ended", flush=True)
    srv.close()


# ===========================================================================
# Main
# ===========================================================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--cam-port", type=int, default=8554)
    ap.add_argument("--mav-port", type=int, default=5772,
                    help="local TCP port to expose for the relay "
                         "(SITL itself uses 5760-5763, so pick something else)")
    ap.add_argument("--sitl-host", default="127.0.0.1")
    ap.add_argument("--sitl-port", type=int, default=5763,
                    help="SITL TCP port. 5760 is usually owned by MAVProxy; "
                         "5762/5763 are spare MAVLink serials.")
    ap.add_argument("--topic", default=None,
                    help="gz image topic; auto-discovered if omitted")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--quality", type=int, default=70)
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-mavlink", action="store_true")
    args = ap.parse_args()

    stop_event = threading.Event()
    threads = []

    if not args.no_camera:
        topic = args.topic or discover_image_topic()
        print(f"[cam ] subscribing to: {topic}", flush=True)
        source = GzCameraSource(topic)
        for _ in range(50):
            if source.get() is not None:
                break
            time.sleep(0.1)
        if source.get() is None:
            print("[cam ] WARNING: no frames received yet", flush=True)
        t = threading.Thread(
            target=serve_camera,
            args=(source, args.host, args.cam_port, args.fps,
                  args.quality, stop_event),
            daemon=True)
        t.start(); threads.append(t)

    if not args.no_mavlink:
        t = threading.Thread(
            target=serve_mavlink,
            args=(args.host, args.mav_port,
                  args.sitl_host, args.sitl_port, stop_event),
            daemon=True)
        t.start(); threads.append(t)

    if not threads:
        print("nothing to do (both --no-camera and --no-mavlink)")
        sys.exit(1)

    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[main] shutting down", flush=True)
        stop_event.set()
        for t in threads:
            t.join(timeout=2.0)


if __name__ == "__main__":
    main()
