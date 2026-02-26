#!/usr/bin/env python3
"""
run_mission.py — Interactive VTOL mission controller.

State machine:
  IDLE     --(arm)------> ARMED    : props spool up over 1 s
  ARMED    --(takeoff)--> AIRBORNE : props spinning, fly to hover altitude
  AIRBORNE --(land)-----> LANDING  : descend to ground
  LANDING  --(auto)-----> IDLE     : 2.5 s after touchdown props spool down

Usage:
  python3 run_mission.py               (Gazebo must be running)
  ros2 launch vtol_description gazebo.launch.py   (then run this in another terminal)
"""

import math
import sys
import time

import gz.transport13 as transport
import gz.msgs10.pose_pb2    as pose_pb2
import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.double_pb2  as double_pb2

# ── constants ────────────────────────────────────────────────────────────────
WORLD   = 'vtol_world'
MODEL   = 'vtol'
SERVICE = f'/world/{WORLD}/set_pose'

GROUND_Z = 0.333         # base_link z at rest — raised 11mm above URDF value
                         # so prop visuals clear the z=0 ground plane (no z-fighting)
HOVER_Z  = GROUND_Z + 1.0

# Roll=+π/2 quaternion  (model Y-up → world Z-up, belly-down)
_h = math.sqrt(2) / 2
QUAT_W, QUAT_X, QUAT_Y, QUAT_Z = _h, _h, 0.0, 0.0

POSE_RATE_HZ   = 60      # Hz for set_pose during flight
SPOOL_UP_S     = 1.0     # seconds to ramp props from 0 → full speed
SPOOL_DOWN_S   = 2.5     # seconds after touchdown before props stop
TAKEOFF_S      = 2.0     # duration of takeoff ramp
LAND_S         = 2.0     # duration of landing ramp

# Joint name → full-speed velocity (rad/s)
# CCW: +50,  CW: -50,  pusher: +30
PROP_JOINTS = [
    ('prop_1_joint', +50.0),
    ('prop_2_joint', -50.0),
    ('prop_3_joint', +50.0),
    ('prop_4_joint', -50.0),
    ('pusher_joint',  +30.0),
]

# ── gz transport ─────────────────────────────────────────────────────────────
_node = transport.Node()

# Publishers for each joint's velocity command
_prop_pubs: dict = {}

def _init_prop_publishers():
    for joint_name, _ in PROP_JOINTS:
        topic = f'/model/{MODEL}/joint/{joint_name}/cmd_vel'
        _prop_pubs[joint_name] = _node.advertise(topic, double_pb2.Double)


# ── pose helpers ─────────────────────────────────────────────────────────────
def set_pose(z, x=0.0, y=0.0, silent=False) -> bool:
    req = pose_pb2.Pose()
    req.name          = MODEL
    req.position.x    = x
    req.position.y    = y
    req.position.z    = z
    req.orientation.w = QUAT_W
    req.orientation.x = QUAT_X
    req.orientation.y = QUAT_Y
    req.orientation.z = QUAT_Z
    ok, _ = _node.request(SERVICE, req, pose_pb2.Pose, boolean_pb2.Boolean, 500)
    if not ok and not silent:
        print('  [warn] set_pose failed', flush=True)
    return ok


def _fly(label: str, z_start: float, z_end: float, duration: float):
    """Smooth linear altitude ramp at POSE_RATE_HZ."""
    dt    = 1.0 / POSE_RATE_HZ
    steps = max(1, round(duration * POSE_RATE_HZ))
    t0    = time.monotonic()
    for i in range(steps + 1):
        alpha   = i / steps
        z       = z_start + alpha * (z_end - z_start)
        elapsed = time.monotonic() - t0
        print(f'\r  [{label}]  {z:.3f} m   ({elapsed:.1f}/{duration:.1f} s)',
              end='', flush=True)
        set_pose(z, silent=True)
        next_tick = t0 + (i + 1) * dt
        gap = next_tick - time.monotonic()
        if gap > 0:
            time.sleep(gap)
    print()


# ── prop helpers ─────────────────────────────────────────────────────────────
def _set_props(scale: float):
    """Publish cmd_vel = max_vel * scale to all prop joints (scale 0..1)."""
    msg = double_pb2.Double()
    for joint_name, max_vel in PROP_JOINTS:
        msg.data = max_vel * scale
        _prop_pubs[joint_name].publish(msg)


def _spool(direction: int, duration: float, label: str):
    """
    Ramp prop velocity.
    direction=+1 : 0 → full speed  (spool up)
    direction=-1 : full speed → 0  (spool down)
    """
    dt    = 1.0 / POSE_RATE_HZ
    steps = max(1, round(duration * POSE_RATE_HZ))
    t0    = time.monotonic()
    for i in range(steps + 1):
        alpha = i / steps
        scale = alpha if direction > 0 else (1.0 - alpha)
        _set_props(scale)
        elapsed = time.monotonic() - t0
        bar = '█' * int(scale * 20) + '░' * (20 - int(scale * 20))
        print(f'\r  [{label}]  [{bar}]  {scale*100:.0f}%   ({elapsed:.1f}/{duration:.1f} s)',
              end='', flush=True)
        next_tick = t0 + (i + 1) * dt
        gap = next_tick - time.monotonic()
        if gap > 0:
            time.sleep(gap)
    print()


# ── state machine ─────────────────────────────────────────────────────────────
def _prompt(valid_cmds: list[str]) -> str:
    """Print prompt, read a command, validate it."""
    options = ' / '.join(f'"{c}"' for c in valid_cmds)
    while True:
        try:
            raw = input(f'\n  > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print('\n[mission] Interrupted.')
            return 'quit'
        if raw in valid_cmds:
            return raw
        if raw == 'quit' or raw == 'q':
            return 'quit'
        print(f'  Unknown command. Available: {options}  (or "quit")')


def run():
    _init_prop_publishers()
    # Give publishers a moment to connect
    time.sleep(0.3)

    print('\n[mission] Waiting for Gazebo + model ...', flush=True)
    # Retry until set_pose service is reachable
    for _ in range(40):
        if set_pose(GROUND_Z, silent=True):
            break
        time.sleep(0.5)
    else:
        print('[mission] ERROR: cannot reach /world/vtol_world/set_pose\n'
              '  Make sure Gazebo is running with world "vtol_world".', file=sys.stderr)
        sys.exit(1)

    print('[mission] Model ready.', flush=True)

    # ── IDLE loop ────────────────────────────────────────────────────────────
    while True:
        print('\n┌─────────────────────────────────────────┐')
        print('│  STATE: IDLE  (propellers stopped)       │')
        print('└─────────────────────────────────────────┘')
        print('  Commands: "arm" | "quit"')

        cmd = _prompt(['arm', 'quit'])
        if cmd == 'quit':
            break

        # ── ARM ──────────────────────────────────────────────────────────────
        print('\n[mission] ▶ ARMING — spooling up propellers ...')
        _spool(+1, SPOOL_UP_S, 'spool-up')
        _set_props(1.0)   # ensure we're exactly at full speed

        print('\n┌─────────────────────────────────────────┐')
        print('│  STATE: ARMED  (propellers spinning)     │')
        print('└─────────────────────────────────────────┘')
        print('  Commands: "takeoff" | "disarm" | "quit"')

        cmd = _prompt(['takeoff', 'disarm', 'quit'])

        if cmd == 'disarm':
            print('\n[mission] ▼ DISARMING — spooling down ...')
            _spool(-1, SPOOL_DOWN_S, 'spool-down')
            _set_props(0.0)
            continue   # back to IDLE

        if cmd == 'quit':
            _set_props(0.0)
            break

        # ── TAKE-OFF ─────────────────────────────────────────────────────────
        print(f'\n[mission] ▲ TAKE-OFF → {HOVER_Z:.2f} m')
        _fly('take-off', GROUND_Z, HOVER_Z, TAKEOFF_S)

        print('\n┌─────────────────────────────────────────┐')
        print('│  STATE: AIRBORNE  (hovering)             │')
        print('└─────────────────────────────────────────┘')
        print(f'  Hovering at {HOVER_Z - GROUND_Z:.1f} m  —  Commands: "land" | "quit"')

        cmd = _prompt(['land', 'quit'])

        if cmd == 'quit':
            _set_props(0.0)
            set_pose(GROUND_Z)
            break

        # ── LAND ─────────────────────────────────────────────────────────────
        print(f'\n[mission] ▼ LANDING → ground')
        _fly('land', HOVER_Z, GROUND_Z, LAND_S)
        set_pose(GROUND_Z)   # snap exactly to ground

        print(f'\n[mission] ✓ Touchdown!  Spooling down in {SPOOL_DOWN_S:.1f} s ...')
        time.sleep(SPOOL_DOWN_S)

        print('[mission] ▼ Spooling down propellers ...')
        _spool(-1, 1.0, 'spool-down')
        _set_props(0.0)
        # Loop back to IDLE

    print('\n[mission] Goodbye.\n')


if __name__ == '__main__':
    run()
