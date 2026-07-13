#!/usr/bin/env bash
# Launch ArduPilot SITL (ArduCopter) wired to Gazebo via the JSON backend.
#
# Start your Gazebo world FIRST (with the ardupilot_gazebo plugin loaded);
# SITL connects to it. This script does NOT start Gazebo or the MAVProxy map.
set -euo pipefail

# --no-extra-ports: stop MAVProxy from adding its default outputs on
# 14550/14551 — they would bypass mavlink-router and collide with the
# gateway's listen port. The single explicit --out feeds the router.
exec sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
    --no-extra-ports \
    --out=udp:127.0.0.1:14550
