#!/usr/bin/env bash
# Run mavlink-router with the project config (config/mavlink-router.conf).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONF="${SCRIPT_DIR}/../config/mavlink-router.conf"

if ! command -v mavlink-routerd >/dev/null 2>&1; then
    echo "error: mavlink-routerd not found." >&2
    echo "Build/install it from https://github.com/mavlink-router/mavlink-router" >&2
    exit 1
fi

exec mavlink-routerd -c "${CONF}"
