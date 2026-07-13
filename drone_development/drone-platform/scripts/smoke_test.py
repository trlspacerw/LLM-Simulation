#!/usr/bin/env python3
"""End-to-end smoke test against the local gateway.

Sequence: link check -> wait GPS -> mode GUIDED -> arm -> takeoff 10 m ->
goto ~30 m north -> RTL. Prints PASS/FAIL per step and exits non-zero on
any failure.

Run with Gazebo, SITL, mavlink-router and the FastAPI server already up:

    python3 scripts/smoke_test.py                 # key from .env / $API_KEY
    python3 scripts/smoke_test.py --api-key XYZ --base-url http://127.0.0.1:8000

Uses only the standard library, so it needs no virtualenv.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple

TAKEOFF_ALT_M = 10.0
GOTO_OFFSET_NORTH_M = 30.0


def load_api_key_from_env_file() -> Optional[str]:
    """Best-effort read of API_KEY from the project .env (no dependency)."""
    env_path = Path(__file__).resolve().parent.parent / ".env"
    if not env_path.is_file():
        return None
    for line in env_path.read_text().splitlines():
        line = line.strip()
        if line.startswith("API_KEY="):
            return line.split("=", 1)[1].strip().strip("'\"") or None
    return None


class Gateway:
    """Tiny stdlib HTTP client for the gateway REST API."""

    def __init__(self, base_url: str, api_key: str, timeout: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key
        self.timeout = timeout

    def _request(self, method: str, path: str,
                 body: Optional[Dict[str, Any]] = None
                 ) -> Tuple[int, Dict[str, Any]]:
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            self.base_url + path, data=data, method=method,
            headers={"Content-Type": "application/json",
                     "X-API-Key": self.api_key})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return resp.status, json.loads(resp.read() or b"{}")
        except urllib.error.HTTPError as exc:
            try:
                payload = json.loads(exc.read() or b"{}")
            except json.JSONDecodeError:
                payload = {}
            return exc.code, payload

    def get(self, path: str) -> Tuple[int, Dict[str, Any]]:
        return self._request("GET", path)

    def post(self, path: str,
             body: Optional[Dict[str, Any]] = None) -> Tuple[int, Dict[str, Any]]:
        return self._request("POST", path, body or {})

    def state(self) -> Dict[str, Any]:
        status, data = self.get("/api/state")
        if status != 200:
            raise RuntimeError(f"GET /api/state -> {status}: {data}")
        return data


def wait_for(predicate: Callable[[Dict[str, Any]], bool], gw: Gateway,
             timeout: float, what: str, poll: float = 1.0) -> Dict[str, Any]:
    """Poll /api/state until predicate(state) is true or timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = gw.state()
        if predicate(state):
            return state
        time.sleep(poll)
    raise TimeoutError(f"timed out after {timeout:.0f}s waiting for {what}")


def horizontal_distance_m(lat1: float, lon1: float,
                          lat2: float, lon2: float) -> float:
    """Equirectangular approximation — plenty for tens of metres."""
    k = 111_111.0  # metres per degree of latitude
    dx = (lon2 - lon1) * k * math.cos(math.radians(lat1))
    dy = (lat2 - lat1) * k
    return math.hypot(dx, dy)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--api-key",
                        default=os.getenv("API_KEY") or load_api_key_from_env_file())
    args = parser.parse_args()
    if not args.api_key:
        print("error: no API key (use --api-key, $API_KEY, or .env)")
        return 2

    gw = Gateway(args.base_url, args.api_key)
    results: list[Tuple[str, bool, str]] = []

    def step(name: str, fn: Callable[[], str]) -> bool:
        try:
            detail = fn()
            results.append((name, True, detail))
            print(f"  PASS  {name}  ({detail})")
            return True
        except Exception as exc:  # report and stop the sequence
            results.append((name, False, str(exc)))
            print(f"  FAIL  {name}: {exc}")
            return False

    def expect_ok(status: int, data: Dict[str, Any], label: str) -> None:
        if status != 200:
            raise RuntimeError(f"{label} -> HTTP {status}: "
                               f"{data.get('detail', data)}")

    # --- the sequence; each lambda returns a short PASS detail string ------
    def s_link() -> str:
        state = wait_for(lambda s: s["link"]["connected"], gw, 15,
                         "MAVLink link up")
        return f"mode={state['mode']}"

    def s_gps() -> str:
        state = wait_for(
            lambda s: (s["gps"]["fix_type"] or 0) >= 3, gw, 60, "3D GPS fix")
        return f"fix={state['gps']['fix_type']} sats={state['gps']['satellites']}"

    def s_guided() -> str:
        expect_ok(*gw.post("/api/mode", {"mode": "GUIDED"}), "POST /api/mode")
        wait_for(lambda s: s["mode"] == "GUIDED", gw, 10, "mode GUIDED")
        return "mode=GUIDED"

    def s_arm() -> str:
        # Pre-arm checks can take a moment to settle after boot; retry.
        last: Exception = RuntimeError("not attempted")
        for attempt in range(5):
            try:
                expect_ok(*gw.post("/api/arm"), "POST /api/arm")
                wait_for(lambda s: s["armed"], gw, 10, "armed")
                return f"armed on attempt {attempt + 1}"
            except Exception as exc:
                last = exc
                time.sleep(5)
        raise RuntimeError(f"arming failed after 5 attempts: {last}")

    def s_takeoff() -> str:
        expect_ok(*gw.post("/api/takeoff", {"altitude": TAKEOFF_ALT_M}),
                  "POST /api/takeoff")
        state = wait_for(
            lambda s: (s["position"]["alt_rel"] or 0) >= TAKEOFF_ALT_M * 0.85,
            gw, 60, f"altitude {TAKEOFF_ALT_M} m")
        return f"alt_rel={state['position']['alt_rel']:.1f} m"

    def s_goto() -> str:
        pos = gw.state()["position"]
        target_lat = pos["lat"] + GOTO_OFFSET_NORTH_M / 111_111.0
        target_lon = pos["lon"]
        expect_ok(*gw.post("/api/goto", {"lat": target_lat, "lon": target_lon,
                                         "alt": TAKEOFF_ALT_M}),
                  "POST /api/goto")
        state = wait_for(
            lambda s: horizontal_distance_m(
                s["position"]["lat"], s["position"]["lon"],
                target_lat, target_lon) < 5.0,
            gw, 90, "arrival within 5 m of goto target", poll=2.0)
        dist = horizontal_distance_m(state["position"]["lat"],
                                     state["position"]["lon"],
                                     target_lat, target_lon)
        return f"reached target ({dist:.1f} m off)"

    def s_rtl() -> str:
        expect_ok(*gw.post("/api/rtl"), "POST /api/rtl")
        wait_for(lambda s: s["mode"] == "RTL", gw, 10, "mode RTL")
        return "mode=RTL"

    steps = [("link up", s_link), ("3D GPS fix", s_gps),
             ("mode GUIDED", s_guided), ("arm", s_arm),
             (f"takeoff {TAKEOFF_ALT_M:.0f} m", s_takeoff),
             (f"goto {GOTO_OFFSET_NORTH_M:.0f} m north", s_goto),
             ("RTL", s_rtl)]

    print(f"Smoke test against {args.base_url}")
    for name, fn in steps:
        if not step(name, fn):
            break

    passed = sum(1 for _, ok, _ in results if ok)
    print(f"\n{passed}/{len(steps)} steps passed")
    return 0 if passed == len(steps) else 1


if __name__ == "__main__":
    sys.exit(main())
