"""API-key authentication and CORS setup.

Every /api/* HTTP request must carry the key in the X-API-Key header; the
WebSocket carries it as the ?api_key= query parameter (browsers cannot set
custom headers on WebSocket handshakes). The key is loaded from .env
(API_KEY) — never hardcoded.
"""
from __future__ import annotations

import logging
import os
import secrets
from typing import List

from fastapi import FastAPI, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

logger = logging.getLogger(__name__)

API_KEY_HEADER = "X-API-Key"
WS_QUERY_PARAM = "api_key"


def get_api_key() -> str:
    """The configured API key; raises if missing so we fail fast at startup."""
    key = os.getenv("API_KEY", "").strip()
    if not key:
        raise RuntimeError(
            "API_KEY is not set. Copy .env.example to .env and set a key "
            "(e.g. `openssl rand -hex 24`).")
    return key


def is_valid_key(candidate: str | None) -> bool:
    """Constant-time comparison against the configured key."""
    return secrets.compare_digest(candidate or "", get_api_key())


def allowed_origins() -> List[str]:
    raw = os.getenv("ALLOWED_ORIGINS", "*")
    return [origin.strip() for origin in raw.split(",") if origin.strip()]


def install(app: FastAPI) -> None:
    """Attach the API-key middleware and CORS to the app.

    Order matters: CORS is added last so it wraps auth and answers preflight
    OPTIONS requests (which carry no custom headers) before auth runs.
    """

    @app.middleware("http")
    async def require_api_key(request: Request, call_next) -> Response:
        if request.url.path.startswith("/api/") and request.method != "OPTIONS":
            if not is_valid_key(request.headers.get(API_KEY_HEADER)):
                return JSONResponse(
                    status_code=401,
                    content={"detail": f"invalid or missing {API_KEY_HEADER}"})
        return await call_next(request)

    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins(),
        allow_methods=["*"],
        allow_headers=["*"],
    )
