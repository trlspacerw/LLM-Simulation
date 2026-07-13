"""High-level vehicle commands exposed as REST endpoints.

All endpoints are plain (non-async) functions so FastAPI runs them in its
thread pool — they block on COMMAND_ACK, never on the event loop. Every
command either returns the ACK result or raises a clear HTTP error:

  401 invalid API key (auth middleware)
  400 bad request (unknown mode, invalid body)
  409 refused: safety guard failed or autopilot rejected (MAV_RESULT_*)
  503 MAVLink link down
  504 no COMMAND_ACK within the timeout
"""
from __future__ import annotations

import logging
from typing import Any, Callable, Dict

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from pymavlink import mavutil

from .mavlink_client import (AckTimeoutError, CommandRejectedError,
                             LinkDownError, MavlinkClient, mav_result_name)
from .telemetry import TelemetryStore

logger = logging.getLogger(__name__)

GPS_FIX_3D = 3  # GPS_FIX_TYPE_3D


class ModeRequest(BaseModel):
    mode: str = Field(min_length=1, description="Flight mode name, e.g. GUIDED")


class TakeoffRequest(BaseModel):
    altitude: float = Field(gt=0, le=120,
                            description="Target altitude above home, metres")


class GotoRequest(BaseModel):
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    alt: float = Field(gt=0, le=120,
                       description="Altitude above home, metres")


def build_router(client: MavlinkClient, store: TelemetryStore) -> APIRouter:
    """Create the /api command router bound to the shared client and store."""
    router = APIRouter(prefix="/api")

    def run(label: str, send: Callable[[], Any]) -> Dict[str, Any]:
        """Run a blocking command, mapping client errors to HTTP errors."""
        try:
            send()
        except CommandRejectedError as exc:
            raise HTTPException(
                status_code=409,
                detail=f"{label} rejected by autopilot: {exc.result_name}")
        except AckTimeoutError as exc:
            raise HTTPException(status_code=504, detail=f"{label}: {exc}")
        except LinkDownError as exc:
            raise HTTPException(status_code=503, detail=str(exc))
        return {"ok": True, "command": label,
                "ack": mav_result_name(mavutil.mavlink.MAV_RESULT_ACCEPTED)}

    def set_mode(mode_name: str) -> Dict[str, Any]:
        mode_name = mode_name.strip().upper()
        try:
            mapping = client.mode_mapping()
        except LinkDownError as exc:
            raise HTTPException(status_code=503, detail=str(exc))
        if mode_name not in mapping:
            raise HTTPException(
                status_code=400,
                detail=f"unknown mode {mode_name!r}; available: "
                       f"{', '.join(sorted(mapping))}")
        return run(f"mode {mode_name}", lambda: client.command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            p2=float(mapping[mode_name])))

    @router.post("/arm")
    def arm() -> Dict[str, Any]:
        return run("arm", lambda: client.command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1.0))

    @router.post("/disarm")
    def disarm() -> Dict[str, Any]:
        return run("disarm", lambda: client.command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=0.0))

    @router.post("/mode")
    def mode(body: ModeRequest) -> Dict[str, Any]:
        return set_mode(body.mode)

    @router.post("/takeoff")
    def takeoff(body: TakeoffRequest) -> Dict[str, Any]:
        # Safety guard: GUIDED + armed + 3D GPS fix, else refuse.
        state = store.snapshot()
        problems = []
        if state["mode"] != "GUIDED":
            problems.append(f"mode is {state['mode']}, need GUIDED")
        if not state["armed"]:
            problems.append("vehicle is not armed")
        fix = state["gps"]["fix_type"]
        if fix is None or fix < GPS_FIX_3D:
            problems.append(f"GPS fix is {fix}, need >= {GPS_FIX_3D} (3D)")
        if problems:
            raise HTTPException(status_code=409,
                                detail="takeoff refused: " + "; ".join(problems))
        return run(f"takeoff {body.altitude:g}m", lambda: client.command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=float(body.altitude)))

    @router.post("/goto")
    def goto(body: GotoRequest) -> Dict[str, Any]:
        state = store.snapshot()
        problems = []
        if state["mode"] != "GUIDED":
            problems.append(f"mode is {state['mode']}, need GUIDED")
        if not state["armed"]:
            problems.append("vehicle is not armed")
        if problems:
            raise HTTPException(status_code=409,
                                detail="goto refused: " + "; ".join(problems))
        result = run("goto", lambda: client.send_position_target_global(
            body.lat, body.lon, body.alt))
        # SET_POSITION_TARGET_GLOBAL_INT has no COMMAND_ACK in MAVLink;
        # "ok" here means the target was sent on a live, GUIDED, armed link.
        result["ack"] = None
        result["note"] = ("position target sent; SET_POSITION_TARGET has no "
                          "COMMAND_ACK — watch position telemetry")
        return result

    @router.post("/rtl")
    def rtl() -> Dict[str, Any]:
        return set_mode("RTL")

    @router.post("/land")
    def land() -> Dict[str, Any]:
        return set_mode("LAND")

    return router
