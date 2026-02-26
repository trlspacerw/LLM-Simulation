#!/usr/bin/env python3
"""
mavlink_controller.py — MAVLink vehicle emulator + Gazebo VTOL controller.

Emulates an ArduPlane/QuadPlane vehicle on MAVLink UDP port 14550.
Accepts standard MAVLink commands to control the vtol_description Gazebo model:

  MAV_CMD_COMPONENT_ARM_DISARM   → props spool up / spool down
  MAV_CMD_DO_SET_MODE + GUIDED   → accepted (always in GUIDED mode)
  MAV_CMD_NAV_VTOL_TAKEOFF       → fly up (param7 = target alt AGL, default 10 m)
  MAV_CMD_NAV_VTOL_LAND          → fly down, props stop 2.5 s after touchdown

Sends back to the GCS:
  HEARTBEAT            (1 Hz)
  GLOBAL_POSITION_INT  (5 Hz)
  EXTENDED_SYS_STATE   (2 Hz)
  COMMAND_ACK          (on every COMMAND_LONG)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  STEP 1 — start Gazebo in another terminal:
    source ~/ardu_ws/install/setup.bash
    ros2 launch vtol_description gazebo.launch.py

  STEP 2 — start this controller (acts as the vehicle):
    python3 mavlink_controller.py

  STEP 3 — connect Mission Planner or run the test script:
    python3 mavlink_test.py          # auto arm → takeoff → hover → land
    Mission Planner: UDP  127.0.0.1 : 14550
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import argparse
import math
import sys
import threading
import time

from pymavlink import mavutil

import gz.transport13    as transport
import gz.msgs10.pose_pb2    as pose_pb2
import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.double_pb2  as double_pb2

# ── Gazebo constants ─────────────────────────────────────────────────────────
WORLD    = 'vtol_world'
MODEL    = 'vtol'
SERVICE  = f'/world/{WORLD}/set_pose'

GROUND_Z      = 0.333      # base_link z at rest — raised 11mm above URDF value
                           # so prop visuals (at base_z-0.323) clear the z=0 ground
                           # plane and avoid z-fighting / flickering.
DEFAULT_ALT_M = 10.0       # default hover altitude AGL (metres)

_h = math.sqrt(2) / 2
QUAT_W, QUAT_X, QUAT_Y, QUAT_Z = _h, _h, 0.0, 0.0   # Roll=+90°, belly-down

POSE_RATE_HZ  = 60         # Hz for set_pose during flight ramps
SPOOL_UP_S    = 1.0        # seconds to ramp props 0 → full
SPOOL_DOWN_S  = 1.0        # seconds to ramp props full → 0 (after touchdown wait)
SPOOL_DOWN_WAIT_S = 2.5    # seconds after touchdown before spool-down starts
CLIMB_SPEED   = 1.5        # m/s for takeoff and landing ramps

PROP_JOINTS = [
    ('prop_1_joint', +50.0),
    ('prop_2_joint', -50.0),
    ('prop_3_joint', +50.0),
    ('prop_4_joint', -50.0),
    ('pusher_joint',  +30.0),
]

# ── MAVLink / telemetry constants ─────────────────────────────────────────────
# ArduPlane GUIDED mode id
GUIDED_MODE = 15

# Reference position for GLOBAL_POSITION_INT (ArduPilot SITL default, Canberra AU)
REF_LAT_DEG = -35.363262
REF_LON_DEG = 149.165237
REF_ALT_MSL = 584.0       # metres MSL

# ── State machine ─────────────────────────────────────────────────────────────
# States
ST_DISARMED  = 'DISARMED'
ST_ARMED     = 'ARMED'
ST_TAKEOFF   = 'TAKEOFF'
ST_HOVER     = 'HOVER'
ST_LANDING   = 'LANDING'
ST_TOUCHDOWN = 'TOUCHDOWN'

# ── Globals shared across threads ─────────────────────────────────────────────
_state      = ST_DISARMED
_state_lock = threading.Lock()
_current_z  = GROUND_Z        # Gazebo model z (updated by flight thread)
_target_alt = DEFAULT_ALT_M   # AGL metres, set on VTOL_TAKEOFF command

_gz_node    = transport.Node()
_prop_pubs  = {}

# Command events sent from MAVLink thread to flight thread
_cmd_arm      = threading.Event()
_cmd_disarm   = threading.Event()
_cmd_takeoff  = threading.Event()
_cmd_land     = threading.Event()


# ── Gazebo helpers ────────────────────────────────────────────────────────────
def _init_prop_publishers():
    for joint_name, _ in PROP_JOINTS:
        topic = f'/model/{MODEL}/joint/{joint_name}/cmd_vel'
        _prop_pubs[joint_name] = _gz_node.advertise(topic, double_pb2.Double)


def _set_props(scale: float):
    msg = double_pb2.Double()
    for joint_name, max_vel in PROP_JOINTS:
        msg.data = max_vel * scale
        _prop_pubs[joint_name].publish(msg)


def _spool(direction: int, duration: float):
    dt    = 1.0 / POSE_RATE_HZ
    steps = max(1, round(duration * POSE_RATE_HZ))
    t0    = time.monotonic()
    for i in range(steps + 1):
        alpha = i / steps
        scale = alpha if direction > 0 else (1.0 - alpha)
        _set_props(scale)
        gap = t0 + (i + 1) * dt - time.monotonic()
        if gap > 0:
            time.sleep(gap)


def _gz_set_pose(z: float, silent=False) -> bool:
    global _current_z
    req = pose_pb2.Pose()
    req.name          = MODEL
    req.position.z    = z
    req.orientation.w = QUAT_W
    req.orientation.x = QUAT_X
    req.orientation.y = QUAT_Y
    req.orientation.z = QUAT_Z
    ok, _ = _gz_node.request(SERVICE, req, pose_pb2.Pose, boolean_pb2.Boolean, 500)
    if ok:
        _current_z = z
    elif not silent:
        print(f'[gz] set_pose failed', flush=True)
    return ok


def _fly_ramp(z_start: float, z_end: float):
    """Ramp altitude from z_start to z_end at CLIMB_SPEED m/s, 60 Hz."""
    dist = abs(z_end - z_start)
    dur  = max(0.5, dist / CLIMB_SPEED)
    dt   = 1.0 / POSE_RATE_HZ
    steps = max(1, round(dur * POSE_RATE_HZ))
    t0   = time.monotonic()
    for i in range(steps + 1):
        alpha = i / steps
        z     = z_start + alpha * (z_end - z_start)
        _gz_set_pose(z, silent=True)
        gap = t0 + (i + 1) * dt - time.monotonic()
        if gap > 0:
            time.sleep(gap)
    _gz_set_pose(z_end, silent=True)


# ── Flight control thread ─────────────────────────────────────────────────────
def _flight_thread():
    global _state, _target_alt

    while True:
        # ── wait for ARM ─────────────────────────────────────────────────────
        _cmd_arm.wait()
        _cmd_arm.clear()
        _cmd_disarm.clear()
        # Do NOT clear _cmd_takeoff here — it may already be set if VTOL_TAKEOFF
        # arrived immediately after ARM (race-safe because recv thread pre-sets
        # _state = ST_ARMED before calling _cmd_arm.set()).
        _cmd_land.clear()

        with _state_lock:
            _state = ST_ARMED

        print('[ctrl] ARMED — spooling up propellers', flush=True)
        _spool(+1, SPOOL_UP_S)
        _set_props(1.0)

        # ── wait for TAKEOFF or DISARM ────────────────────────────────────
        print('[ctrl] Waiting for TAKEOFF or DISARM command ...', flush=True)
        while True:
            if _cmd_disarm.is_set():
                _cmd_disarm.clear()
                with _state_lock:
                    _state = ST_DISARMED
                print('[ctrl] DISARMED — spooling down', flush=True)
                _spool(-1, SPOOL_DOWN_S)
                _set_props(0.0)
                break

            if _cmd_takeoff.is_set():
                _cmd_takeoff.clear()
                hover_z = GROUND_Z + _target_alt

                with _state_lock:
                    _state = ST_TAKEOFF

                print(f'[ctrl] TAKEOFF → {_target_alt:.1f} m AGL  '
                      f'(z = {hover_z:.3f} m)', flush=True)
                _fly_ramp(GROUND_Z, hover_z)

                with _state_lock:
                    _state = ST_HOVER

                print(f'[ctrl] HOVER at {_target_alt:.1f} m AGL. '
                      'Waiting for LAND ...', flush=True)

                # ── wait for LAND ─────────────────────────────────────────
                while True:
                    if _cmd_land.is_set():
                        _cmd_land.clear()
                        break
                    time.sleep(0.05)

                with _state_lock:
                    _state = ST_LANDING

                print('[ctrl] LANDING → ground', flush=True)
                _fly_ramp(hover_z, GROUND_Z)
                _gz_set_pose(GROUND_Z)

                with _state_lock:
                    _state = ST_TOUCHDOWN

                print(f'[ctrl] Touchdown.  Props stop in {SPOOL_DOWN_WAIT_S} s ...',
                      flush=True)
                time.sleep(SPOOL_DOWN_WAIT_S)

                print('[ctrl] Spooling down propellers ...', flush=True)
                _spool(-1, SPOOL_DOWN_S)
                _set_props(0.0)

                with _state_lock:
                    _state = ST_DISARMED

                print('[ctrl] DISARMED. Ready for next ARM.', flush=True)
                break   # back to outer wait-for-ARM loop

            time.sleep(0.02)

        # If we broke out due to disarm, loop back to wait-for-ARM
        continue


# ── MAVLink helpers ───────────────────────────────────────────────────────────
def _ack(mav, cmd_id: int, result: int):
    mav.mav.command_ack_send(cmd_id, result)


def _base_mode() -> int:
    """Compute MAVLink base_mode flags from current state."""
    flags = (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
             mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED)
    with _state_lock:
        armed = _state != ST_DISARMED
    if armed:
        flags |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return flags


def _mav_state() -> int:
    with _state_lock:
        s = _state
    if s == ST_DISARMED:
        return mavutil.mavlink.MAV_STATE_STANDBY
    if s in (ST_TAKEOFF, ST_LANDING, ST_TOUCHDOWN):
        return mavutil.mavlink.MAV_STATE_ACTIVE
    return mavutil.mavlink.MAV_STATE_ACTIVE


def _landed_state() -> int:
    with _state_lock:
        s = _state
    if s == ST_DISARMED:
        return mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    if s == ST_ARMED:
        return mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    if s == ST_TAKEOFF:
        return mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF
    if s == ST_HOVER:
        return mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
    if s == ST_LANDING:
        return mavutil.mavlink.MAV_LANDED_STATE_LANDING
    if s == ST_TOUCHDOWN:
        return mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    return mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED


# ── Telemetry sender thread ───────────────────────────────────────────────────
def _telemetry_thread(mav):
    t_hb  = 0.0
    t_pos = 0.0
    t_ext = 0.0

    while True:
        now = time.monotonic()

        # HEARTBEAT @ 1 Hz
        if now - t_hb >= 1.0:
            t_hb = now
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                _base_mode(),
                GUIDED_MODE,
                _mav_state(),
            )

        # GLOBAL_POSITION_INT @ 5 Hz
        if now - t_pos >= 0.2:
            t_pos = now
            rel_alt_m = _current_z - GROUND_Z          # AGL metres
            alt_msl_m = REF_ALT_MSL + rel_alt_m
            mav.mav.global_position_int_send(
                int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
                int(REF_LAT_DEG * 1e7),                # lat (1e-7 deg)
                int(REF_LON_DEG * 1e7),                # lon (1e-7 deg)
                int(alt_msl_m * 1000),                 # alt MSL (mm)
                int(rel_alt_m * 1000),                 # relative_alt (mm)
                0, 0, 0,                                # vx, vy, vz (cm/s)
                0xFFFF,                                 # hdg (unknown)
            )

        # EXTENDED_SYS_STATE @ 2 Hz
        if now - t_ext >= 0.5:
            t_ext = now
            mav.mav.extended_sys_state_send(
                mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
                _landed_state(),
            )

        time.sleep(0.01)


# ── MAVLink receive thread ────────────────────────────────────────────────────
def _mavlink_recv_thread(mav):
    global _state, _target_alt

    print('[mav] Listening for commands ...', flush=True)

    while True:
        msg = mav.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue

        mtype = msg.get_type()

        # ── SET_MODE ─────────────────────────────────────────────────────────
        if mtype == 'SET_MODE':
            # Always accept — we're always in GUIDED
            mav.mav.command_ack_send(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
            )
            print(f'[mav] SET_MODE accepted (custom_mode={msg.custom_mode})',
                  flush=True)
            continue

        if mtype != 'COMMAND_LONG':
            continue

        cmd = msg.command

        # ── ARM / DISARM ──────────────────────────────────────────────────────
        if cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            arm_flag = int(msg.param1)
            with _state_lock:
                cur = _state

            if arm_flag == 1:   # ARM
                if cur != ST_DISARMED:
                    _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
                    print('[mav] ARM rejected — already armed', flush=True)
                else:
                    # Pre-set state now so a fast VTOL_TAKEOFF isn't rejected
                    with _state_lock:
                        _state = ST_ARMED
                    _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                    print('[mav] ARM accepted', flush=True)
                    _cmd_arm.set()

            else:               # DISARM
                if cur not in (ST_ARMED, ST_DISARMED):
                    _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
                    print('[mav] DISARM rejected — vehicle is in the air', flush=True)
                else:
                    _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                    print('[mav] DISARM accepted', flush=True)
                    _cmd_disarm.set()

        # ── VTOL TAKEOFF ──────────────────────────────────────────────────────
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF:
            with _state_lock:
                cur = _state

            if cur != ST_ARMED:
                _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
                print(f'[mav] VTOL_TAKEOFF rejected — state is {cur}', flush=True)
            else:
                alt = float(msg.param7)
                if alt <= 0:
                    alt = DEFAULT_ALT_M
                _target_alt = alt
                _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                print(f'[mav] VTOL_TAKEOFF accepted — target {alt:.1f} m AGL',
                      flush=True)
                _cmd_takeoff.set()

        # ── VTOL LAND ─────────────────────────────────────────────────────────
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND:
            with _state_lock:
                cur = _state

            if cur not in (ST_HOVER, ST_TAKEOFF):
                _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED)
                print(f'[mav] VTOL_LAND rejected — state is {cur}', flush=True)
            else:
                _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                print('[mav] VTOL_LAND accepted', flush=True)
                _cmd_land.set()

        # ── DO_SET_MODE ───────────────────────────────────────────────────────
        elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)

        # ── everything else → accepted silently ───────────────────────────────
        else:
            _ack(mav, cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)


# ── main ─────────────────────────────────────────────────────────────────────
def main(connection_str: str):
    global _current_z

    # ── Connect to Gazebo ─────────────────────────────────────────────────────
    print('[gz] Initialising gz.transport publishers ...', flush=True)
    _init_prop_publishers()
    time.sleep(0.5)   # let publishers advertise

    print('[gz] Waiting for Gazebo set_pose service ...', flush=True)
    for _ in range(40):
        if _gz_set_pose(GROUND_Z, silent=True):
            break
        time.sleep(0.5)
    else:
        print('[gz] ERROR: cannot reach /world/vtol_world/set_pose.\n'
              '  Start Gazebo first:  ros2 launch vtol_description gazebo.launch.py',
              file=sys.stderr)
        sys.exit(1)

    _current_z = GROUND_Z
    print('[gz] Gazebo connected.', flush=True)

    # ── Open MAVLink connection ───────────────────────────────────────────────
    print(f'[mav] Opening MAVLink on {connection_str} ...', flush=True)
    mav = mavutil.mavlink_connection(
        connection_str,
        source_system=1,
        source_component=1,
    )
    mav.mav.srcSystem    = 1
    mav.mav.srcComponent = 1

    print()
    print('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
    print('  MAVLink vehicle ready on', connection_str)
    print('  Connect Mission Planner  →  TCP  127.0.0.1 : 5760')
    print('  Or run:  python3 mavlink_test.py --connect tcp:127.0.0.1:5760')
    print('  (Waiting for GCS to connect ...)')
    print()
    print('  Expected command sequence:')
    print('    1. Set mode → GUIDED')
    print('    2. ARM')
    print('    3. MAV_CMD_NAV_VTOL_TAKEOFF  (param7 = altitude AGL)')
    print('    4. MAV_CMD_NAV_VTOL_LAND')
    print('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

    # Start threads immediately so heartbeats go out before any GCS connects.
    # (Do NOT block here waiting for GCS — that would prevent heartbeat TX.)
    threading.Thread(target=_flight_thread,       daemon=True).start()
    threading.Thread(target=_telemetry_thread,    args=(mav,), daemon=True).start()
    threading.Thread(target=_mavlink_recv_thread, args=(mav,), daemon=True).start()

    print('[ctrl] Flight thread running.  Waiting for ARM ...', flush=True)

    # Main thread: keep alive
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('\n[ctrl] Shutting down — stopping props.', flush=True)
        _set_props(0.0)
        _gz_set_pose(GROUND_Z)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='MAVLink-controlled Gazebo VTOL controller'
    )
    parser.add_argument(
        '--listen',
        default='tcpin:0.0.0.0:5760',
        help='MAVLink connection string (default: tcpin:0.0.0.0:5760)\n'
             'Examples:\n'
             '  tcpin:0.0.0.0:5760     ← TCP listen server on port 5760 (default)\n'
             '  udpin:0.0.0.0:14550    ← UDP server on port 14550',
    )
    args = parser.parse_args()
    main(args.listen)
