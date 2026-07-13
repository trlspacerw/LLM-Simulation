"""FastAPI entry point: wires MAVLink client, telemetry, commands and auth.

Run from the project root (so .env is found):

    uvicorn server.main:app --host 0.0.0.0 --port 8000
"""
from __future__ import annotations

import asyncio
import logging
import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, AsyncIterator, Dict

from dotenv import load_dotenv

load_dotenv()  # must run before auth reads API_KEY

from fastapi import FastAPI, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

from . import auth
from .commands import build_router
from .mavlink_client import MavlinkClient
from .telemetry import TelemetryStore, WebSocketManager, run_broadcaster

logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO"),
    format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger(__name__)

client = MavlinkClient(os.getenv("MAVLINK_ENDPOINT", "udpin:0.0.0.0:14551"))
store = TelemetryStore()
ws_manager = WebSocketManager()


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    auth.get_api_key()  # fail fast if .env / API_KEY is missing
    client.add_message_listener(store.handle_message)
    client.add_link_listener(store.handle_link)
    client.start()
    broadcaster = asyncio.create_task(run_broadcaster(store, ws_manager))
    logger.info("gateway up; waiting for MAVLink heartbeat")
    try:
        yield
    finally:
        broadcaster.cancel()
        client.stop()


app = FastAPI(title="Drone Platform Gateway", lifespan=lifespan)
auth.install(app)
app.include_router(build_router(client, store))


@app.get("/api/state")
def get_state() -> Dict[str, Any]:
    """Latest telemetry snapshot, for polling clients."""
    return store.snapshot()


DASHBOARD = Path(__file__).resolve().parent.parent / "client" / "index.html"


@app.get("/", include_in_schema=False)
def dashboard() -> FileResponse:
    """Serve the single-file dashboard, so one URL gives UI + API.

    The page itself is public; everything it calls still requires the key.
    """
    return FileResponse(DASHBOARD, media_type="text/html")


@app.get("/healthz")
def healthz() -> Dict[str, Any]:
    """Unauthenticated liveness probe (no telemetry data)."""
    return {"ok": True, "mavlink_connected": client.connected}


@app.websocket("/ws/telemetry")
async def ws_telemetry(websocket: WebSocket,
                       api_key: str = Query(default="")) -> None:
    """Telemetry stream: state at 2 Hz plus immediate status/link events."""
    if not auth.is_valid_key(api_key):
        # 4401: app-specific "unauthorized" close code (HTTP 401 equivalent).
        await websocket.close(code=4401, reason="invalid api_key")
        return
    await ws_manager.connect(websocket)
    try:
        # Push the current state immediately so the UI renders at once.
        await websocket.send_json({"type": "state", "data": store.snapshot()})
        while True:
            # Browsers don't send anything; this just detects disconnects.
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        ws_manager.disconnect(websocket)
