#!/usr/bin/env bash
# Start Gazebo + two independent ArduCopter SITL instances.
#
# No ROS. Each drone is fully independent:
#   Drone 1 (iris)   SITL inst 0  MAVLink tcp 5760  FDM 9002
#   Drone 2 (iris2)  SITL inst 1  MAVLink tcp 5770  FDM 9012
#
# Stop everything with Ctrl+C (cleanup trap below).

set -e

WS=/home/trlspace/ardu_ws
source "$WS/install/setup.bash"

WORLD="$WS/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds/iris_tracking_two.sdf"
PARM_BASE="$WS/install/ardupilot_gazebo/share/ardupilot_gazebo/config/gazebo-iris-gimbal.parm"
LOG_DIR=/tmp/two_drones
mkdir -p "$LOG_DIR"

# ---- cleanup -----------------------------------------------------
PIDS=()
cleanup() {
    echo
    echo "[launcher] shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "arducopter" 2>/dev/null || true
    echo "[launcher] done."
}
trap cleanup EXIT INT TERM

# ---- Gazebo server + GUI -----------------------------------------
echo "[launcher] starting gz sim with $(basename "$WORLD")"
gz sim -v4 -s -r "$WORLD" > "$LOG_DIR/gz-server.log" 2>&1 &
PIDS+=($!)
gz sim -v4 -g > "$LOG_DIR/gz-gui.log" 2>&1 &
PIDS+=($!)

# wait for world to be alive
for i in $(seq 1 20); do
    sleep 1
    if gz topic -l 2>/dev/null | grep -q "/world/iris_tracking_two"; then
        echo "[launcher] gz sim ready"
        break
    fi
done

# ---- ArduCopter SITL instance 0 (SYSID 1) ------------------------
echo "[launcher] starting arducopter instance 0 (drone 1, SYSID 1)"
arducopter \
    --model json \
    --speedup 1 \
    --instance 0 \
    --sysid 1 \
    --sim-address 127.0.0.1 \
    --defaults "$PARM_BASE" \
    > "$LOG_DIR/sitl0.log" 2>&1 &
PIDS+=($!)

# ---- ArduCopter SITL instance 1 (SYSID 2) ------------------------
echo "[launcher] starting arducopter instance 1 (drone 2, SYSID 2)"
arducopter \
    --model json \
    --speedup 1 \
    --instance 1 \
    --sysid 2 \
    --sim-address 127.0.0.1 \
    --defaults "$PARM_BASE" \
    > "$LOG_DIR/sitl1.log" 2>&1 &
PIDS+=($!)

cat <<EOF

[launcher] all up. logs in $LOG_DIR/

  Drone 1: MAVLink  tcp:127.0.0.1:5760    (also 5762, 5763)
           Camera   /world/iris_tracking_two/model/iris/link/front_camera_link/sensor/front_camera/image

  Drone 2: MAVLink  tcp:127.0.0.1:5770    (also 5772, 5773)
           Camera   /world/iris_tracking_two/model/iris2/link/front_camera_link/sensor/front_camera/image

Test:
  python3 $WS/src/drone_development/connect_two_drones.py

Press Ctrl+C to stop everything.
EOF

# wait on any of the children (or signal)
wait
