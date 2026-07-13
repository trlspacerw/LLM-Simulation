"""Latest-state telemetry store and WebSocket broadcaster.

TelemetryStore is fed from the MAVLink receive thread (handle_message /
handle_link) and read from the asyncio side (snapshot / drain_events), so all
access goes through a lock. run_broadcaster() pushes:

  * {"type": "status", ...}  STATUSTEXT events, forwarded within 100 ms
  * {"type": "link", ...}    link up/down transitions
  * {"type": "state", ...}   the full latest-state dict at 2 Hz
"""
from __future__ import annotations

import asyncio
import collections
import copy
import logging
import threading
import time
from typing import Any, Deque, Dict, List, Set

from fastapi import WebSocket
from pymavlink import mavutil

logger = logging.getLogger(__name__)

STATE_PERIOD_S = 0.5   # full-state broadcast period (2 Hz)
EVENT_POLL_S = 0.1     # how often queued events are flushed


def _initial_state() -> Dict[str, Any]:
    return {
        "link": {"connected": False, "last_heartbeat": None},
        "mode": None,
        "armed": False,
        "position": {"lat": None, "lon": None, "alt_msl": None,
                     "alt_rel": None, "heading": None},
        "vfr": {"airspeed": None, "groundspeed": None,
                "climb": None, "throttle": None},
        "gps": {"fix_type": None, "satellites": None, "hdop": None},
        "battery": {"voltage": None, "current": None, "remaining": None},
        "last_statustext": None,
    }


class TelemetryStore:
    """Thread-safe latest-state dict plus a queue of one-shot events."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = _initial_state()
        self._events: Deque[Dict[str, Any]] = collections.deque(maxlen=200)

    # Called from the MAVLink receive thread for every message.
    def handle_message(self, msg: object) -> None:
        mtype = msg.get_type()
        with self._lock:
            state = self._state
            if mtype == "HEARTBEAT":
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    return
                state["mode"] = mavutil.mode_string_v10(msg)
                state["armed"] = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                state["link"]["last_heartbeat"] = time.time()
            elif mtype == "GLOBAL_POSITION_INT":
                state["position"] = {
                    "lat": msg.lat / 1e7,
                    "lon": msg.lon / 1e7,
                    "alt_msl": msg.alt / 1000.0,
                    "alt_rel": msg.relative_alt / 1000.0,
                    "heading": msg.hdg / 100.0 if msg.hdg != 65535 else None,
                }
            elif mtype == "VFR_HUD":
                state["vfr"] = {
                    "airspeed": msg.airspeed,
                    "groundspeed": msg.groundspeed,
                    "climb": msg.climb,
                    "throttle": msg.throttle,
                }
            elif mtype == "SYS_STATUS":
                # mV -> V, cA -> A; -1/255 mean "not provided".
                state["battery"] = {
                    "voltage": msg.voltage_battery / 1000.0
                    if msg.voltage_battery != 65535 else None,
                    "current": msg.current_battery / 100.0
                    if msg.current_battery != -1 else None,
                    "remaining": msg.battery_remaining
                    if msg.battery_remaining != -1 else None,
                }
            elif mtype == "GPS_RAW_INT":
                state["gps"] = {
                    "fix_type": msg.fix_type,
                    "satellites": msg.satellites_visible
                    if msg.satellites_visible != 255 else None,
                    "hdop": msg.eph / 100.0 if msg.eph != 65535 else None,
                }
            elif mtype == "BATTERY_STATUS":
                # Refines SYS_STATUS with per-battery data when present.
                if msg.voltages and msg.voltages[0] != 65535:
                    state["battery"]["voltage"] = msg.voltages[0] / 1000.0
                if msg.battery_remaining != -1:
                    state["battery"]["remaining"] = msg.battery_remaining
            elif mtype == "STATUSTEXT":
                entry = {"severity": msg.severity, "text": msg.text,
                         "time": time.time()}
                state["last_statustext"] = entry
                self._events.append({"type": "status", **entry})

    # Called from the MAVLink RX thread on link up/down transitions.
    def handle_link(self, connected: bool) -> None:
        with self._lock:
            self._state["link"]["connected"] = connected
            self._events.append({"type": "link", "connected": connected,
                                 "time": time.time()})

    def snapshot(self) -> Dict[str, Any]:
        """Deep copy of the latest state, safe to serialize."""
        with self._lock:
            return copy.deepcopy(self._state)

    def drain_events(self) -> List[Dict[str, Any]]:
        """Pop all queued one-shot events (STATUSTEXT, link transitions)."""
        with self._lock:
            events = list(self._events)
            self._events.clear()
        return events


class WebSocketManager:
    """Tracks connected WebSocket clients and broadcasts JSON payloads."""

    def __init__(self) -> None:
        self._clients: Set[WebSocket] = set()

    def has_clients(self) -> bool:
        return bool(self._clients)

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._clients.add(websocket)
        logger.info("WebSocket client connected (%d total)",
                    len(self._clients))

    def disconnect(self, websocket: WebSocket) -> None:
        self._clients.discard(websocket)
        logger.info("WebSocket client disconnected (%d left)",
                    len(self._clients))

    async def broadcast(self, payload: Dict[str, Any]) -> None:
        dead: List[WebSocket] = []
        for client in list(self._clients):
            try:
                await client.send_json(payload)
            except Exception:
                dead.append(client)
        for client in dead:
            self._clients.discard(client)


async def run_broadcaster(store: TelemetryStore,
                          manager: WebSocketManager) -> None:
    """Forever: flush events promptly, broadcast the full state at 2 Hz."""
    next_state = 0.0
    while True:
        events = store.drain_events()
        if manager.has_clients():
            for event in events:
                await manager.broadcast(event)
            now = time.monotonic()
            if now >= next_state:
                await manager.broadcast(
                    {"type": "state", "data": store.snapshot()})
                next_state = now + STATE_PERIOD_S
        await asyncio.sleep(EVENT_POLL_S)
