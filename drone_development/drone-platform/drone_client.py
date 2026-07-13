#!/usr/bin/env python3
"""Connect a locally running simulation to the web platform.

Use this when you start Gazebo/SITL yourself, e.g.:

    # terminal 1 — your simulation
    cd ~/ardu_ws && source install/setup.bash
    ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=true

    # terminal 2 — your MAVProxy (console + map), feeding the platform
    mavproxy.py --master=udp:127.0.0.1:14550 \
                --out=127.0.0.1:14551 \
                --out=tcpin:0.0.0.0:5761 \
                --console --map

    # terminal 3 — this script: web gateway + ngrok tunnels
    python3 drone_client.py            # add --no-ngrok for local-only

The gateway listens on UDP 14551 (your MAVProxy --out feeds it) and serves
the dashboard at http://127.0.0.1:8000. The optional tcpin:5761 output is
what the ngrok TCP tunnel forwards to remote QGroundControl — without it
only the web path works. mavlink-router is NOT needed in this mode.

Stop with Ctrl+C (stops the gateway and ngrok; your sim keeps running).
"""
from __future__ import annotations

import argparse
import atexit
import json
import os
import secrets
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional

PROJECT_DIR = Path(__file__).resolve().parent
NGROK_API = "http://127.0.0.1:4040/api/tunnels"


def _get_json(url: str) -> Dict[str, Any]:
    with urllib.request.urlopen(url, timeout=2) as resp:
        return json.loads(resp.read())


def ensure_env() -> str:
    """Create .env with a fresh API key on first run; return the key."""
    env_path = PROJECT_DIR / ".env"
    if not env_path.is_file():
        template = (PROJECT_DIR / ".env.example").read_text()
        env_path.write_text(template.replace(
            "API_KEY=", f"API_KEY={secrets.token_hex(24)}", 1))
        print("created .env with a new API key")
    for line in env_path.read_text().splitlines():
        if line.startswith("API_KEY=") and line.split("=", 1)[1].strip():
            return line.split("=", 1)[1].strip()
    sys.exit("error: API_KEY is empty in .env — set it and rerun")


def start_ngrok() -> Optional[subprocess.Popen]:
    """Start both tunnels via scripts/start_ngrok.sh; None if it can't run."""
    log = (PROJECT_DIR / "logs" / "ngrok.log").open("w")
    proc = subprocess.Popen(
        ["bash", str(PROJECT_DIR / "scripts" / "start_ngrok.sh")],
        stdout=log, stderr=subprocess.STDOUT)
    atexit.register(proc.terminate)
    return proc


def status_reporter(api_key: str, port: int, ngrok_enabled: bool) -> None:
    """Background thread: print URLs once ready, then watch the link."""
    for _ in range(30):  # wait for uvicorn to come up
        try:
            _get_json(f"http://127.0.0.1:{port}/healthz")
            break
        except Exception:
            time.sleep(1)

    lines = [f"  dashboard : http://127.0.0.1:{port}",
             f"  API key   : {api_key}"]
    if ngrok_enabled:
        tunnels: list = []
        for _ in range(20):
            try:
                tunnels = _get_json(NGROK_API)["tunnels"]
                if tunnels:
                    break
            except Exception:
                pass
            time.sleep(1)
        if tunnels:
            lines += [f"  {t['name']:8s}  : {t['public_url']}" for t in tunnels]
        else:
            lines.append("  tunnels   : NOT UP — authtoken set? see logs/ngrok.log")
    # flush=True: this thread's output must not sit in the buffer when
    # stdout is piped or redirected to a log file.
    print("\n=== web platform ready ===", flush=True)
    print("\n".join(lines), flush=True)
    print("\nwaiting for MAVLink from your MAVProxy "
          "(--out=127.0.0.1:14551)...\n", flush=True)

    last: Optional[bool] = None
    while True:
        try:
            connected = _get_json(
                f"http://127.0.0.1:{port}/healthz")["mavlink_connected"]
        except Exception:
            return  # gateway is shutting down
        if connected != last:
            print("vehicle link:",
                  "UP — telemetry flowing" if connected else
                  "DOWN — is MAVProxy running with --out=127.0.0.1:14551?",
                  flush=True)
            last = connected
        time.sleep(2)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--no-ngrok", action="store_true",
                        help="local only, skip the tunnels")
    parser.add_argument("--port", type=int, default=8000,
                        help="gateway port (the ngrok http tunnel expects 8000)")
    args = parser.parse_args()

    os.chdir(PROJECT_DIR)  # so .env and server/ resolve
    (PROJECT_DIR / "logs").mkdir(exist_ok=True)
    os.environ["PATH"] = f"{Path.home() / '.local/bin'}:{os.environ['PATH']}"
    api_key = ensure_env()

    ngrok_proc = None if args.no_ngrok else start_ngrok()
    threading.Thread(target=status_reporter,
                     args=(api_key, args.port, ngrok_proc is not None),
                     daemon=True).start()

    import uvicorn
    uvicorn.run("server.main:app", host="0.0.0.0", port=args.port,
                log_level="warning")


if __name__ == "__main__":
    main()
