#!/usr/bin/env bash
# One-command startup for the whole platform:
#   Gazebo (headless by default) -> SITL -> mavlink-router -> FastAPI gateway [-> ngrok]
#
# Usage:
#   scripts/start_all.sh             # local: dashboard at http://127.0.0.1:8000
#   scripts/start_all.sh --gui       # show the Gazebo GUI
#   scripts/start_all.sh --ngrok     # also start ngrok and print the public URLs
#   scripts/start_all.sh <world.sdf> # use a different Gazebo world
#
# Stop everything with scripts/stop_all.sh. Logs land in logs/.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
PROJECT_DIR="$(pwd)"
LOG_DIR="${PROJECT_DIR}/logs"
export PATH="${HOME}/.local/bin:${PATH}"
mkdir -p "${LOG_DIR}"

GUI=false; NGROK=false; WORLD="iris_runway.sdf"
for arg in "$@"; do
    case "${arg}" in
        --gui)   GUI=true ;;
        --ngrok) NGROK=true ;;
        *)       WORLD="${arg}" ;;
    esac
done

# Refuse to double-start.
if ss -tuln | grep -qE ':(8000|14550)\b'; then
    echo "error: platform ports already in use — run scripts/stop_all.sh first" >&2
    exit 1
fi

# First run: create .env with a fresh random API key.
if [[ ! -f .env ]]; then
    cp .env.example .env
    sed -i "s/^API_KEY=.*/API_KEY=$(openssl rand -hex 24)/" .env
    echo "created .env with a new API key"
fi
API_KEY="$(grep '^API_KEY=' .env | cut -d= -f2-)"

: > "${LOG_DIR}/pids"
launch() {  # launch <name> <command...>; records the pid for stop_all.sh
    local name="$1"; shift
    nohup "$@" > "${LOG_DIR}/${name}.log" 2>&1 &
    echo "$! ${name}" >> "${LOG_DIR}/pids"
    echo "  ${name} (pid $!, log logs/${name}.log)"
}

echo "[1/4] Gazebo world: ${WORLD}"
if ${GUI}; then
    launch gazebo gz sim -r "${WORLD}"
else
    launch gazebo gz sim -s -r "${WORLD}"
fi

echo "[2/4] ArduPilot SITL"
SIM_VEHICLE="$(command -v sim_vehicle.py)" || {
    echo "error: sim_vehicle.py not on PATH" >&2; exit 1; }
ARDUPILOT_ROOT="$(cd "$(dirname "${SIM_VEHICLE}")/../.." && pwd)"
cd "${ARDUPILOT_ROOT}/ArduCopter"
launch sitl python3 "${SIM_VEHICLE}" -v ArduCopter -f gazebo-iris --model JSON \
    --no-extra-ports --out=udp:127.0.0.1:14550 --mavproxy-args=--daemon
cd "${PROJECT_DIR}"

echo "[3/4] mavlink-router"
launch router mavlink-routerd -c "${PROJECT_DIR}/config/mavlink-router.conf"

echo "[4/4] FastAPI gateway"
launch gateway python3 -m uvicorn server.main:app --host 0.0.0.0 --port 8000

printf "waiting for the MAVLink link (first SITL build can take a minute)"
LINK=""
for _ in $(seq 1 180); do
    if curl -sf http://127.0.0.1:8000/healthz 2>/dev/null \
            | grep -q '"mavlink_connected":true'; then
        LINK=ok; break
    fi
    printf "."; sleep 1
done
echo
if [[ "${LINK}" != ok ]]; then
    echo "error: MAVLink link did not come up in 180 s — check logs/*.log" >&2
    echo "(then run scripts/stop_all.sh before retrying)" >&2
    exit 1
fi

echo
echo "platform is UP"
echo "  dashboard : http://127.0.0.1:8000   (API key below)"
echo "  API key   : ${API_KEY}"
echo "  note      : wait ~20-30 s for GPS/EKF before arming/takeoff"
echo "  stop      : scripts/stop_all.sh"

if ${NGROK}; then
    echo
    echo "[5/5] ngrok tunnels"
    launch ngrok bash "${PROJECT_DIR}/scripts/start_ngrok.sh"
    sleep 5
    curl -s http://127.0.0.1:4040/api/tunnels | python3 -c '
import json, sys
try:
    tunnels = json.load(sys.stdin)["tunnels"]
    for t in tunnels:
        print(f"  {t[\"name\"]:8s}: {t[\"public_url\"]}")
    if not tunnels:
        raise ValueError
except Exception:
    print("  tunnels not ready — check logs/ngrok.log or http://127.0.0.1:4040")
'
fi
