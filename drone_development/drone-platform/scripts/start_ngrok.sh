#!/usr/bin/env bash
# Start both ngrok tunnels (tcp 5761 for MAVLink, http 8000 for the API).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}/.."
CONF="${PROJECT_DIR}/config/ngrok.yml"

if ! command -v ngrok >/dev/null 2>&1; then
    echo "error: ngrok not found. Install: https://ngrok.com/download" >&2
    exit 1
fi

# Authtoken resolution, in order of preference:
#   1. NGROK_AUTHTOKEN from the project .env (or already exported)
#   2. the default agent config written by `ngrok config add-authtoken`
if [[ -f "${PROJECT_DIR}/.env" ]]; then
    set -a; source "${PROJECT_DIR}/.env"; set +a
fi

ARGS=()
if [[ -z "${NGROK_AUTHTOKEN:-}" ]]; then
    DEFAULT_CFG="${HOME}/.config/ngrok/ngrok.yml"
    if [[ -f "${DEFAULT_CFG}" ]]; then
        # Merge the default config (holds the authtoken) with our endpoints.
        # If ngrok complains about a config version mismatch, run:
        #   ngrok config upgrade
        ARGS+=(--config "${DEFAULT_CFG}")
    else
        echo "warning: no NGROK_AUTHTOKEN and no ${DEFAULT_CFG}." >&2
        echo "Run: ngrok config add-authtoken <YOUR_TOKEN>" >&2
    fi
fi
ARGS+=(--config "${CONF}")

echo "Assigned addresses: see the ngrok UI below, or:"
echo "  curl -s http://127.0.0.1:4040/api/tunnels | python3 -m json.tool"
exec ngrok start --all "${ARGS[@]}"
