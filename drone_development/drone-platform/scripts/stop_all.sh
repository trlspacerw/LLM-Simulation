#!/usr/bin/env bash
# Stop everything started by start_all.sh (and any stray platform processes).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

if [[ -f logs/pids ]]; then
    while read -r pid name; do
        kill "${pid}" 2>/dev/null && echo "stopped ${name} (pid ${pid})"
    done < logs/pids
    rm -f logs/pids
fi
sleep 1

# Backstop: sim_vehicle/gz spawn children the pid file doesn't cover.
for proc in arducopter mavproxy.py mavlink-routerd ngrok; do
    pkill -x "${proc}" 2>/dev/null && echo "killed stray ${proc}"
done
pkill -f "gz sim" 2>/dev/null && echo "killed stray gz sim"
pkill -f "uvicorn server.main" 2>/dev/null && echo "killed stray gateway"

# arducopter in particular ignores SIGTERM; escalate to SIGKILL if needed.
for _ in 1 2 3 4 5; do
    sleep 1
    ss -tuln | grep -qE ':(5760|5761|8000|14550|14551)\b' || break
done
for proc in arducopter mavproxy.py mavlink-routerd ngrok; do
    pkill -9 -x "${proc}" 2>/dev/null && echo "force-killed ${proc}"
done
sleep 1

if ss -tuln | grep -E ':(5760|5761|8000|14550|14551)\b'; then
    echo "warning: some ports are still in use (see above)" >&2
    exit 1
fi
echo "all platform ports free"
