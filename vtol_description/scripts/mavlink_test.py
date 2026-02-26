#!/usr/bin/env python3
"""
mavlink_test.py — MAVLink test script for the ArduPlane QuadPlane SITL.

Connects to the running ArduPilot SITL, arms the VTOL, commands a vertical
takeoff to 10 m, hovers for 5 seconds, then lands — all over MAVLink.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  STEP 1 — start the SITL simulation in another terminal:
    source ~/ardu_ws/install/setup.bash
    ros2 launch drone_nl_control nl_drone.launch.py vehicle:=quadplane

  STEP 2 — run this script:
    python3 mavlink_test.py                          # default: udp:127.0.0.1:14550
    python3 mavlink_test.py --connect udp::14550     # same
    python3 mavlink_test.py --connect tcp:127.0.0.1:5762  # secondary TCP

  MISSION PLANNER (connect from same or different machine):
    Connection type : TCP  → host: 127.0.0.1  port: 5760   (primary)
                      UDP  → host: 127.0.0.1  port: 14550  (GCS broadcast)
    If Mission Planner is on a different PC, replace 127.0.0.1 with this
    machine's LAN IP (e.g. 192.168.1.x).
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import argparse
import sys
import time

from pymavlink import mavutil

# ── mission parameters ───────────────────────────────────────────────────────
TAKEOFF_ALT_M   = 10.0   # relative altitude for VTOL takeoff (metres)
HOVER_SECONDS   = 5      # seconds to hover before landing
ARM_TIMEOUT     = 15     # seconds to wait for arming ACK
CMD_TIMEOUT     = 15     # seconds to wait for command ACK
HEARTBEAT_TOUT  = 30     # seconds to wait for first heartbeat


# ── helpers ──────────────────────────────────────────────────────────────────

def connect(connection_str: str):
    """Open MAVLink connection and wait for the first heartbeat."""
    print(f'\n[mav] Connecting to {connection_str} ...', flush=True)
    mav = mavutil.mavlink_connection(connection_str)
    print(f'[mav] Waiting for heartbeat (up to {HEARTBEAT_TOUT} s)...', flush=True)
    mav.wait_heartbeat(timeout=HEARTBEAT_TOUT)
    print(f'[mav] ✓ Heartbeat received from '
          f'system={mav.target_system}  component={mav.target_component}',
          flush=True)
    return mav


def get_mode_id(mav, name: str) -> int:
    """Return the numeric mode ID for a named flight mode."""
    mapping = mav.mode_mapping()
    if name not in mapping:
        available = ', '.join(sorted(mapping.keys()))
        sys.exit(f'[mav] ERROR: mode "{name}" not found.\nAvailable: {available}')
    return mapping[name]


def set_mode(mav, mode_name: str):
    mode_id = get_mode_id(mav, mode_name)
    print(f'[mav] Setting mode → {mode_name} ({mode_id})', flush=True)
    mav.set_mode(mode_id)
    # wait for heartbeat confirming mode change
    for _ in range(10):
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            print(f'[mav] ✓ Mode confirmed: {mode_name}', flush=True)
            return
    print(f'[mav] ⚠  Mode change not confirmed (continuing anyway)', flush=True)


def arm(mav):
    print('[mav] Arming motors...', flush=True)
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,      # confirmation
        1,      # arm=1 / disarm=0
        0, 0, 0, 0, 0, 0
    )
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=ARM_TIMEOUT)
    if ack is None:
        sys.exit('[mav] ERROR: no ARM ACK received.')
    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        result_str = mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
        sys.exit(f'[mav] ERROR: ARM rejected — {result_str}.\n'
                 '  Make sure the SITL is fully initialised and GPS is locked.')
    print('[mav] ✓ Armed', flush=True)


def send_command(mav, cmd_id: int, params: list, label: str):
    """Send a MAV_CMD and wait for ACK."""
    p = (params + [0] * 7)[:7]
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        cmd_id,
        0,           # confirmation
        p[0], p[1], p[2], p[3], p[4], p[5], p[6]
    )
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=CMD_TIMEOUT)
    if ack is None:
        print(f'[mav] ⚠  No ACK for {label} (command may still execute)', flush=True)
        return False
    result_str = mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
    ok = ack.result in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS)
    symbol = '✓' if ok else '✗'
    print(f'[mav] {symbol} {label} ACK → {result_str}', flush=True)
    return ok


def wait_altitude(mav, target_m: float, tolerance: float = 1.5, timeout: float = 60):
    """Block until relative altitude is within `tolerance` m of `target_m`."""
    print(f'[mav] Climbing to {target_m:.1f} m (timeout {timeout} s)...', flush=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            alt = msg.relative_alt / 1000.0   # mm → m
            print(f'\r  altitude: {alt:6.2f} m  (target {target_m:.1f} m)',
                  end='', flush=True)
            if abs(alt - target_m) < tolerance:
                print(f'\n[mav] ✓ Target altitude reached: {alt:.2f} m', flush=True)
                return True
    print(f'\n[mav] ⚠  Altitude timeout — last reading may be close enough',
          flush=True)
    return False


def hover(mav, seconds: int):
    print(f'[mav] Hovering for {seconds} s...', flush=True)
    end = time.time() + seconds
    while time.time() < end:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            alt = msg.relative_alt / 1000.0
            rem = end - time.time()
            print(f'\r  alt={alt:.2f} m  |  {rem:.0f} s remaining   ',
                  end='', flush=True)
    print(flush=True)


def wait_landed(mav, timeout: float = 60):
    """Wait until EXTENDED_SYS_STATE reports landed."""
    print('[mav] Waiting for touchdown...', flush=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = mav.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=2)
        if msg:
            # MAV_LANDED_STATE: 0=UNDEFINED, 1=ON_GROUND, 2=IN_AIR, 3=TAKEOFF, 4=LANDING
            if msg.landed_state == 1:
                print('[mav] ✓ Landed', flush=True)
                return True
        # fallback: also check altitude
        pos = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos and pos.relative_alt / 1000.0 < 0.5:
            print('[mav] ✓ Near ground (alt < 0.5 m)', flush=True)
            return True
    print('[mav] ⚠  Land timeout', flush=True)
    return False


def print_vehicle_info(mav):
    """Print basic vehicle info from heartbeat."""
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if not hb:
        return
    vtype = mavutil.mavlink.enums['MAV_TYPE'].get(hb.type, None)
    vname = vtype.name if vtype else str(hb.type)
    atype = mavutil.mavlink.enums['MAV_AUTOPILOT'].get(hb.autopilot, None)
    aname = atype.name if atype else str(hb.autopilot)
    modes = mav.mode_mapping()
    cur_mode = [k for k, v in modes.items() if v == hb.custom_mode]
    print(f'[mav] Vehicle  : {vname}')
    print(f'[mav] Autopilot: {aname}')
    print(f'[mav] Mode     : {cur_mode[0] if cur_mode else hb.custom_mode}')


# ── main mission ─────────────────────────────────────────────────────────────

def run_mission(connection_str: str):
    mav = connect(connection_str)

    print()
    print_vehicle_info(mav)

    # ── 1. Set GUIDED mode ───────────────────────────────────────────────────
    print()
    set_mode(mav, 'GUIDED')

    # ── 2. Arm ───────────────────────────────────────────────────────────────
    arm(mav)

    # ── 3. VTOL Takeoff ──────────────────────────────────────────────────────
    print(f'\n[mav] ══ TAKE-OFF to {TAKEOFF_ALT_M} m ══')
    send_command(
        mav,
        mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
        # param1=0 (no specific heading), param2-6=unused, param7=altitude AGL
        [0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M],
        label='VTOL_TAKEOFF'
    )
    wait_altitude(mav, TAKEOFF_ALT_M)

    # ── 4. Hover ─────────────────────────────────────────────────────────────
    print(f'\n[mav] ══ HOVER {HOVER_SECONDS} s ══')
    hover(mav, HOVER_SECONDS)

    # ── 5. VTOL Land ─────────────────────────────────────────────────────────
    print('\n[mav] ══ LAND ══')
    send_command(
        mav,
        mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
        [0, 0, 0, 0, 0, 0, 0],
        label='VTOL_LAND'
    )
    wait_landed(mav)

    print('\n[mav] ══ MISSION COMPLETE ══\n')
    mav.close()


# ── entry point ──────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='MAVLink VTOL test: arm → takeoff → hover → land'
    )
    parser.add_argument(
        '--connect',
        default='udp:127.0.0.1:14550',
        help='MAVLink connection string  (default: udp:127.0.0.1:14550)\n'
             'Examples:\n'
             '  udp:127.0.0.1:14550  ← GCS broadcast port (default)\n'
             '  tcp:127.0.0.1:5760   ← primary SITL port\n'
             '  tcp:127.0.0.1:5762   ← secondary SITL TCP port',
    )
    parser.add_argument(
        '--alt',
        type=float,
        default=TAKEOFF_ALT_M,
        help=f'Takeoff altitude in metres (default: {TAKEOFF_ALT_M})'
    )
    parser.add_argument(
        '--hover',
        type=int,
        default=HOVER_SECONDS,
        help=f'Hover duration in seconds (default: {HOVER_SECONDS})'
    )
    args = parser.parse_args()

    TAKEOFF_ALT_M  = args.alt
    HOVER_SECONDS  = args.hover

    run_mission(args.connect)
