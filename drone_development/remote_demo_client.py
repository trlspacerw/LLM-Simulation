#!/usr/bin/env python3
"""
Minimal remote-side example that uses the unified gazebo_streamer.py.

Connects to:
  * the MAVLink TCP relay (--mav-host:--mav-port) and waits for a heartbeat,
    reads telemetry for a few seconds, then disarms-safe (no flight commands)
  * the camera TCP stream (--cam-host:--cam-port) and grabs one frame, saving
    it to snapshot.jpg

Run on the remote machine after launching:
    python3 gazebo_streamer.py            # on sim host
    ngrok tcp 8554                        # on sim host (camera)
    ngrok tcp 5772                        # on sim host (mavlink)

Then on remote:
    python3 remote_demo_client.py \\
        --mav-host 0.tcp.ngrok.io --mav-port 12345 \\
        --cam-host 0.tcp.ngrok.io --cam-port 23456
"""

import argparse
import socket
import struct
import time

from pymavlink import mavutil


def grab_one_frame(host: str, port: int, out_path: str = "snapshot.jpg"):
    print(f"[cam ] connecting to {host}:{port}")
    sock = socket.create_connection((host, port), timeout=10)
    try:
        hdr = b""
        while len(hdr) < 4:
            chunk = sock.recv(4 - len(hdr))
            if not chunk:
                raise RuntimeError("disconnected before frame header")
            hdr += chunk
        (size,) = struct.unpack(">I", hdr)
        payload = b""
        while len(payload) < size:
            chunk = sock.recv(size - len(payload))
            if not chunk:
                raise RuntimeError("disconnected mid-frame")
            payload += chunk
        with open(out_path, "wb") as f:
            f.write(payload)
        print(f"[cam ] saved {out_path}  ({size/1024:.1f} KB)")
    finally:
        sock.close()


def read_telemetry(host: str, port: int, seconds: float = 5.0):
    uri = f"tcp:{host}:{port}"
    print(f"[mav ] connecting {uri}")
    m = mavutil.mavlink_connection(uri)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("no heartbeat — relay or SITL not reachable")
    print(f"[mav ] heartbeat: sys={m.target_system} comp={m.target_component} "
          f"type={hb.type} autopilot={hb.autopilot}")

    end = time.time() + seconds
    counts: dict[str, int] = {}
    last_gps = None
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1
        if t == "GLOBAL_POSITION_INT":
            last_gps = msg

    print(f"[mav ] received {sum(counts.values())} msgs in {seconds:.0f}s; "
          f"top types:")
    for k, v in sorted(counts.items(), key=lambda kv: -kv[1])[:6]:
        print(f"       {k:24s} {v}")
    if last_gps is not None:
        print(f"[mav ] last GPS: lat={last_gps.lat/1e7:.6f} "
              f"lon={last_gps.lon/1e7:.6f} alt={last_gps.alt/1000:.1f} m")
    m.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav-host", required=True)
    ap.add_argument("--mav-port", type=int, required=True)
    ap.add_argument("--cam-host", required=True)
    ap.add_argument("--cam-port", type=int, required=True)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--snapshot", default="snapshot.jpg")
    args = ap.parse_args()

    grab_one_frame(args.cam_host, args.cam_port, args.snapshot)
    read_telemetry(args.mav_host, args.mav_port, args.seconds)
    print("done.")


if __name__ == "__main__":
    main()
