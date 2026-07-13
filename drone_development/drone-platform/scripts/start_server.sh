#!/usr/bin/env bash
# Run the FastAPI MAVLink gateway with uvicorn.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

if [[ ! -f .env ]]; then
    echo "error: .env not found. Copy .env.example to .env and set API_KEY." >&2
    exit 1
fi

# Pick up HOST/PORT from .env so the script and the app agree.
set -a; source .env; set +a

exec uvicorn server.main:app --host "${HOST:-0.0.0.0}" --port "${PORT:-8000}"
