#!/usr/bin/env python3
"""
Receive JPEG frames from gazebo_camera_stream.py over TCP and display them.

Usage (local test):
    python3 remote_camera_viewer.py --host 127.0.0.1 --port 8554

Usage (through ngrok):
    # on sim host:  ngrok tcp 8554   ->  e.g. tcp://0.tcp.ngrok.io:12345
    python3 remote_camera_viewer.py --host 0.tcp.ngrok.io --port 12345

Press 'q' in the window to quit. Use --no-show to run headless and just
inspect throughput (useful when the remote server has no display).
"""

import argparse
import socket
import struct
import sys
import time

import cv2
import numpy as np


def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save", default=None,
                    help="optional path to write an MP4 of the stream")
    args = ap.parse_args()

    print(f"[viewer] connecting to {args.host}:{args.port} ...")
    sock = socket.create_connection((args.host, args.port), timeout=10)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("[viewer] connected")

    writer = None
    frames = 0
    t0 = time.time()

    try:
        while True:
            hdr = recv_exact(sock, 4)
            if hdr is None:
                print("[viewer] server closed connection")
                break
            (size,) = struct.unpack(">I", hdr)
            if size == 0 or size > 50_000_000:
                print(f"[viewer] bad frame size {size}; aborting")
                break
            payload = recv_exact(sock, size)
            if payload is None:
                print("[viewer] truncated frame; aborting")
                break

            frame = cv2.imdecode(np.frombuffer(payload, np.uint8),
                                 cv2.IMREAD_COLOR)
            if frame is None:
                continue
            frames += 1

            if args.save:
                if writer is None:
                    h, w = frame.shape[:2]
                    writer = cv2.VideoWriter(
                        args.save, cv2.VideoWriter_fourcc(*"mp4v"),
                        15.0, (w, h))
                writer.write(frame)

            if not args.no_show:
                cv2.imshow("gazebo cam", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            if time.time() - t0 >= 5.0:
                print(f"[viewer] {frames/(time.time()-t0):.1f} fps  "
                      f"last frame {len(payload)/1024:.1f} KB")
                frames = 0
                t0 = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
