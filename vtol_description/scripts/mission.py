#!/usr/bin/env python3
"""
mission.py — Automatic VTOL takeoff → cruise → hover → land mission.

Connects directly to Gazebo via gz.transport (no MAVLink, no ROS topics).
The model climbs to a configurable altitude, optionally cruises forward,
hovers, then lands — fully automatic, no key-presses required.

Anti-flicker design
───────────────────
A background *pose-streaming thread* runs at STREAM_HZ (60 Hz) and is the
ONLY code that calls set_pose.  The motion ramp loop only writes to shared
_target_x/_target_y/_target_z floats — it never calls set_pose itself.

  • Service latency never accumulates into the ramp timer.
  • The model is kept pinned to GROUND_Z while on the ground.
  • Each service call uses a short timeout (80 ms).

Coordinate system
─────────────────
  URDF model frame:  X=lateral, Y=up, Z=nose(+Z=forward)
  Gazebo world frame: X=east, Y=north, Z=up
  Spawn rotation:     Roll=+π/2 → model Y maps to world Z (up),
                      model Z (nose) maps to world -Y.
  Therefore: "forward" = −Y in Gazebo world coordinates.

Usage
─────
  # Gazebo must already be running:
  ros2 launch vtol_description gazebo.launch.py

  # Then, in a second terminal:
  source ~/ardu_ws/install/setup.bash
  python3 mission.py                          # 5 m, 10 s hover, 1.5 m/s
  python3 mission.py --alt 15                 # climb to 15 m
  python3 mission.py --alt 10 --hover 30      # 10 m, hold 30 s
  python3 mission.py --alt 8 --speed 2.5      # fast climb/descent
  python3 mission.py --alt 5 --avoidance      # with obstacle avoidance
  python3 mission.py --alt 5 --forward 20     # cruise 20 m forward
  python3 mission.py --forward 30 --cruise-speed 3.0 --avoidance
"""

import argparse
import math
import os
import sys
import threading
import time

import gz.transport13    as transport
import gz.msgs10.pose_pb2    as pose_pb2
import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.double_pb2  as double_pb2

# ── world / model constants ────────────────────────────────────────────────────
WORLD   = 'vtol_world'
MODEL   = 'vtol'
SERVICE = f'/world/{WORLD}/set_pose'

# GROUND_Z: base_link z at rest.
# Raised 11 mm above the URDF spawn value so the lowest prop visual (~z-0.323)
# clears z=0 by ≥ 10 mm — eliminates ground-plane z-fighting flicker.
GROUND_Z = 0.333

# Roll = +π/2 quaternion: model Y-up → world Z-up, belly pointing down.
_h = math.sqrt(2) / 2
_QUAT = (float(_h), float(_h), 0.0, 0.0)  # (w, x, y, z)

# ── motion / timing constants ──────────────────────────────────────────────────
STREAM_HZ        = 60     # pose stream rate (Hz) — controls visual smoothness
STREAM_TIMEOUT_MS = 80    # max ms per set_pose call; short so stream never stalls
SPOOL_UP_S       = 1.5    # prop spool-up duration (seconds)
SPOOL_DOWN_S     = 1.5    # prop spool-down duration (seconds)
TOUCHDOWN_WAIT_S = 2.5    # pause on ground before spool-down

PROP_JOINTS = [
    ('prop_1_joint', +50.0),
    ('prop_2_joint', -50.0),
    ('prop_3_joint', +50.0),
    ('prop_4_joint', -50.0),
    ('pusher_joint',  +30.0),
]

# ── obstacle avoidance ────────────────────────────────────────────────────────
HALT_FILE = '/tmp/vtol_obstacle_halt'
_avoidance_enabled = False   # set True by --avoidance flag
_last_halt_state   = False   # for change-detection logging


def _check_halt() -> bool:
    """Return True if obstacle avoidance says HALT (file contains '1')."""
    if not _avoidance_enabled:
        return False
    try:
        with open(HALT_FILE, 'r') as f:
            return f.read().strip() == '1'
    except OSError:
        return False


# ── shared state (ramp thread writes; pose thread reads) ──────────────────────
_target_x  = 0.0           # Gazebo x (lateral)
_target_y  = 0.0           # Gazebo y (forward axis: nose = -Y)
_target_z  = GROUND_Z      # Gazebo z (up)
_pose_lock = threading.Lock()
_streaming = False          # True while we need the pose thread to run
_node      = transport.Node()
_prop_pubs: dict = {}


# ── gz helpers ─────────────────────────────────────────────────────────────────

def _init_props():
    for name, _ in PROP_JOINTS:
        topic = f'/model/{MODEL}/joint/{name}/cmd_vel'
        _prop_pubs[name] = _node.advertise(topic, double_pb2.Double)


def _set_props(scale: float):
    msg = double_pb2.Double()
    for name, max_vel in PROP_JOINTS:
        msg.data = max_vel * float(scale)
        _prop_pubs[name].publish(msg)


def _make_pose_msg(x: float, y: float, z: float) -> pose_pb2.Pose:
    p = pose_pb2.Pose()
    p.name          = MODEL
    p.position.x    = float(x)
    p.position.y    = float(y)
    p.position.z    = float(z)
    p.orientation.w = _QUAT[0]
    p.orientation.x = _QUAT[1]
    p.orientation.y = _QUAT[2]
    p.orientation.z = _QUAT[3]
    return p


def _set_pose_once(x: float, y: float, z: float, timeout_ms: int = 400) -> bool:
    ok, _ = _node.request(
        SERVICE, _make_pose_msg(x, y, z),
        pose_pb2.Pose, boolean_pb2.Boolean, timeout_ms
    )
    return ok


# ── pose-streaming thread ──────────────────────────────────────────────────────
# Runs for the entire duration of the mission (including ground-idle).
# Keeps the model locked to _target_z at STREAM_HZ regardless of service speed.

def _pose_stream_thread():
    dt = 1.0 / STREAM_HZ
    while True:
        # Idle: model is on the ground but we still stream to suppress z-drift
        if not _streaming:
            with _pose_lock:
                x, y = _target_x, _target_y
            _set_pose_once(x, y, GROUND_Z, timeout_ms=200)
            time.sleep(0.1)     # 10 Hz is plenty while idle
            continue

        t0 = time.monotonic()
        with _pose_lock:
            x, y, z = _target_x, _target_y, _target_z

        # Short timeout: abandon slow calls instantly so the stream stays timely
        _set_pose_once(x, y, z, timeout_ms=STREAM_TIMEOUT_MS)

        elapsed = time.monotonic() - t0
        gap = dt - elapsed
        if gap > 0:
            time.sleep(gap)
        # gap < 0 → call was slow; loop immediately to send the updated position


# ── prop spool ramp ────────────────────────────────────────────────────────────

def _spool(direction: int, duration: float, label: str):
    steps = max(1, round(duration * STREAM_HZ))
    t0 = time.monotonic()
    for i in range(steps + 1):
        alpha = i / steps
        scale = alpha if direction > 0 else 1.0 - alpha
        _set_props(scale)
        bar = '█' * int(scale * 24) + '░' * (24 - int(scale * 24))
        print(f'\r  [{label}]  [{bar}]  {scale*100:3.0f}%', end='', flush=True)
        gap = t0 + (i + 1) / STREAM_HZ - time.monotonic()
        if gap > 0:
            time.sleep(gap)
    print()


# ── altitude ramp ─────────────────────────────────────────────────────────────
# Only updates _target_z — the streaming thread does the actual Gazebo calls.

def _ramp(label: str, z_start: float, z_end: float, speed_mps: float):
    global _target_z, _last_halt_state
    dist     = abs(z_end - z_start)
    duration = max(0.5, dist / speed_mps)
    steps    = max(1, round(duration * STREAM_HZ))
    t0       = time.monotonic()

    i = 0
    while i <= steps:
        # ── obstacle avoidance: freeze ramp while halted ──
        if _check_halt():
            if not _last_halt_state:
                print(f'\n  [HALT] Obstacle detected — pausing {label}', flush=True)
                _last_halt_state = True
            time.sleep(0.1)
            # Adjust t0 so elapsed time doesn't include paused time
            t0 = time.monotonic() - (i / STREAM_HZ)
            continue
        if _last_halt_state:
            print(f'\n  [CLEAR] Resuming {label}', flush=True)
            _last_halt_state = False

        alpha = i / steps
        z     = z_start + alpha * (z_end - z_start)
        with _pose_lock:
            _target_z = z
        agl = z - GROUND_Z
        elapsed = time.monotonic() - t0
        print(f'\r  [{label}]  z={z:.3f} m   AGL={agl:.2f} m   '
              f'({elapsed:.1f} / {duration:.1f} s)', end='', flush=True)
        gap = t0 + (i + 1) / STREAM_HZ - time.monotonic()
        if gap > 0:
            time.sleep(gap)
        i += 1

    # Hard-set final position to avoid rounding error
    with _pose_lock:
        _target_z = z_end
    print()


def _cruise(label: str, y_start: float, y_end: float, speed_mps: float):
    """Move along the Y axis (forward = -Y in Gazebo world frame)."""
    global _target_y, _last_halt_state
    dist     = abs(y_end - y_start)
    duration = max(0.5, dist / speed_mps)
    steps    = max(1, round(duration * STREAM_HZ))
    t0       = time.monotonic()

    i = 0
    while i <= steps:
        # ── obstacle avoidance: freeze cruise while halted ──
        if _check_halt():
            if not _last_halt_state:
                print(f'\n  [HALT] Obstacle detected — pausing {label}', flush=True)
                _last_halt_state = True
            time.sleep(0.1)
            t0 = time.monotonic() - (i / STREAM_HZ)
            continue
        if _last_halt_state:
            print(f'\n  [CLEAR] Resuming {label}', flush=True)
            _last_halt_state = False

        alpha = i / steps
        y     = y_start + alpha * (y_end - y_start)
        with _pose_lock:
            _target_y = y
        travelled = abs(y - y_start)
        with _pose_lock:
            agl = _target_z - GROUND_Z
        elapsed = time.monotonic() - t0
        print(f'\r  [{label}]  fwd={travelled:.1f}/{dist:.1f} m   '
              f'AGL={agl:.1f} m   ({elapsed:.1f} / {duration:.1f} s)',
              end='', flush=True)
        gap = t0 + (i + 1) / STREAM_HZ - time.monotonic()
        if gap > 0:
            time.sleep(gap)
        i += 1

    with _pose_lock:
        _target_y = y_end
    print()


# ── main mission ───────────────────────────────────────────────────────────────

def run(alt_m: float, hover_s: float, speed_mps: float,
        forward_m: float = 0.0, cruise_speed_mps: float = 0.0):
    global _streaming, _target_x, _target_y, _target_z, _last_halt_state

    hover_z = GROUND_Z + alt_m
    has_cruise = forward_m > 0.0
    if cruise_speed_mps <= 0.0:
        cruise_speed_mps = speed_mps   # default to climb/land speed

    # Phase numbering depends on whether cruise is included
    n_phases = 5 if has_cruise else 4

    # ── init gz transport ──────────────────────────────────────────────────────
    print('[mission] Initialising gz.transport ...', flush=True)
    _init_props()
    time.sleep(0.3)

    # ── wait for Gazebo set_pose service ─────────────────────────────────────
    print('[mission] Waiting for Gazebo (vtol_world) ...', flush=True)
    for attempt in range(40):
        if _set_pose_once(0.0, 0.0, GROUND_Z):
            break
        if attempt % 4 == 0:
            print(f'  (attempt {attempt + 1}/40 — is Gazebo running?)', flush=True)
        time.sleep(0.5)
    else:
        print(
            '\n[mission] ERROR: cannot reach /world/vtol_world/set_pose\n'
            '  Start Gazebo first:\n'
            '    source ~/ardu_ws/install/setup.bash\n'
            '    ros2 launch vtol_description gazebo.launch.py',
            file=sys.stderr,
        )
        sys.exit(1)

    with _pose_lock:
        _target_x = 0.0
        _target_y = 0.0
        _target_z = GROUND_Z

    # ── start background pose thread ──────────────────────────────────────────
    threading.Thread(target=_pose_stream_thread, daemon=True).start()

    # ── print mission summary ─────────────────────────────────────────────────
    print()
    print('┌─────────────────────────────────────────────────────┐')
    print('│              VTOL MISSION PARAMETERS                 │')
    print('├─────────────────────────────────────────────────────┤')
    print(f'│  Takeoff altitude : {alt_m:5.1f} m AGL                      │')
    if has_cruise:
        print(f'│  Forward cruise   : {forward_m:5.1f} m @ {cruise_speed_mps:.1f} m/s             │')
    print(f'│  Hover duration   : {hover_s:5.1f} s                         │')
    print(f'│  Climb/land speed : {speed_mps:5.1f} m/s                      │')
    print(f'│  Pose stream rate : {STREAM_HZ} Hz (anti-flicker thread)    │')
    if _avoidance_enabled:
        print(f'│  Obstacle avoid.  : ENABLED                          │')
    print('└─────────────────────────────────────────────────────┘')
    print()
    print('  Ctrl+C at any time to abort (model returns to ground).')
    print()

    phase = 0

    # ── ARM ────────────────────────────────────────────────────────────────────
    phase += 1
    print('─' * 55)
    print(f'[mission]  PHASE {phase}/{n_phases} — ARM  (spooling up propellers)')
    print('─' * 55)
    _streaming = True
    _spool(+1, SPOOL_UP_S, 'spool-up')
    _set_props(1.0)
    print('[mission]  ✓ Armed\n')

    # ── TAKE-OFF ───────────────────────────────────────────────────────────────
    phase += 1
    print('─' * 55)
    print(f'[mission]  PHASE {phase}/{n_phases} — TAKEOFF  → {alt_m:.1f} m AGL')
    print('─' * 55)
    _ramp('takeoff', GROUND_Z, hover_z, speed_mps)
    print(f'[mission]  ✓ Reached {alt_m:.1f} m AGL\n')

    # ── CRUISE FORWARD (optional) ─────────────────────────────────────────────
    if has_cruise:
        phase += 1
        # Forward = -Y in Gazebo world frame (nose points along -Y after spawn rotation)
        with _pose_lock:
            y_start = _target_y
        y_end = y_start - forward_m   # negative because forward = -Y
        print('─' * 55)
        print(f'[mission]  PHASE {phase}/{n_phases} — CRUISE FORWARD  {forward_m:.1f} m @ {cruise_speed_mps:.1f} m/s')
        print('─' * 55)
        _cruise('cruise', y_start, y_end, cruise_speed_mps)
        print(f'[mission]  ✓ Cruised {forward_m:.1f} m forward\n')

    # ── HOVER ──────────────────────────────────────────────────────────────────
    phase += 1
    print('─' * 55)
    print(f'[mission]  PHASE {phase}/{n_phases} — HOVER  ({hover_s:.0f} s)')
    print('─' * 55)
    t_end = time.monotonic() + hover_s
    while time.monotonic() < t_end:
        if _check_halt():
            if not _last_halt_state:
                print(f'\n  [HALT] Obstacle detected during hover — holding position', flush=True)
                _last_halt_state = True
        elif _last_halt_state:
            print(f'\n  [CLEAR] Obstacle cleared during hover', flush=True)
            _last_halt_state = False
        rem = t_end - time.monotonic()
        with _pose_lock:
            z = _target_z
        halt_tag = ' [HALTED]' if _last_halt_state else ''
        print(f'\r  [hover]  z={z:.3f} m   AGL={alt_m:.2f} m   '
              f'{rem:.0f} s remaining{halt_tag}   ', end='', flush=True)
        time.sleep(0.2)
    print()
    print('[mission]  ✓ Hover complete\n')

    # ── LAND ───────────────────────────────────────────────────────────────────
    phase += 1
    print('─' * 55)
    print(f'[mission]  PHASE {phase}/{n_phases} — LAND')
    print('─' * 55)
    _ramp('land', hover_z, GROUND_Z, speed_mps)
    with _pose_lock:
        _target_z = GROUND_Z
    print('[mission]  ✓ Touchdown!\n')

    # ── DISARM ─────────────────────────────────────────────────────────────────
    print(f'[mission]  Waiting {TOUCHDOWN_WAIT_S:.0f} s before spool-down ...')
    t_end = time.monotonic() + TOUCHDOWN_WAIT_S
    while time.monotonic() < t_end:
        print(f'\r  [on ground]  {t_end - time.monotonic():.0f} s  ', end='', flush=True)
        time.sleep(0.2)
    print()

    print('[mission]  DISARM — spooling down propellers ...')
    _spool(-1, SPOOL_DOWN_S, 'spool-down')
    _set_props(0.0)

    _streaming = False     # pose thread reverts to 10 Hz idle mode

    print()
    print('┌─────────────────────────────────────────────────────┐')
    print('│              ✓  MISSION COMPLETE                     │')
    print('└─────────────────────────────────────────────────────┘')
    print()


# ── abort handler ──────────────────────────────────────────────────────────────

def _abort():
    print('\n[mission]  Abort — stopping props and returning to ground ...', flush=True)
    _set_props(0.0)
    with _pose_lock:
        x, y = _target_x, _target_y
    _set_pose_once(x, y, GROUND_Z, timeout_ms=1000)
    print('[mission]  Safe.', flush=True)


# ── entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Automatic VTOL mission: takeoff → cruise → hover → land',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Examples:\n'
            '  python3 mission.py                                  # 5 m, 10 s hover\n'
            '  python3 mission.py --alt 15                         # climb to 15 m\n'
            '  python3 mission.py --alt 10 --hover 30              # 10 m, 30 s hover\n'
            '  python3 mission.py --alt 8 --speed 3.0              # fast climb\n'
            '  python3 mission.py --forward 20                     # cruise 20 m forward\n'
            '  python3 mission.py --forward 30 --cruise-speed 3.0  # fast cruise\n'
            '  python3 mission.py --forward 20 --avoidance         # cruise with avoidance\n'
        ),
    )
    parser.add_argument(
        '--alt', type=float, default=5.0, metavar='METRES',
        help='Takeoff altitude AGL in metres  (default: 5.0)',
    )
    parser.add_argument(
        '--hover', type=float, default=10.0, metavar='SECONDS',
        help='Hover duration in seconds  (default: 10.0)',
    )
    parser.add_argument(
        '--speed', type=float, default=1.5, metavar='M_PER_S',
        help='Climb and descent speed in m/s  (default: 1.5)',
    )
    parser.add_argument(
        '--forward', type=float, default=0.0, metavar='METRES',
        help='Forward cruise distance in metres (default: 0 = no cruise)',
    )
    parser.add_argument(
        '--cruise-speed', type=float, default=0.0, metavar='M_PER_S',
        help='Cruise speed in m/s (default: same as --speed)',
    )
    parser.add_argument(
        '--avoidance', action='store_true',
        help='Enable obstacle avoidance (reads /tmp/vtol_obstacle_halt)',
    )
    args = parser.parse_args()

    if args.alt <= 0:
        parser.error('--alt must be > 0')
    if args.hover <= 0:
        parser.error('--hover must be > 0')
    if args.speed <= 0:
        parser.error('--speed must be > 0')
    if args.forward < 0:
        parser.error('--forward must be >= 0')

    if args.avoidance:
        _avoidance_enabled = True
        print('[mission] Obstacle avoidance ENABLED — polling', HALT_FILE)

    try:
        run(args.alt, args.hover, args.speed,
            forward_m=args.forward, cruise_speed_mps=args.cruise_speed)
    except KeyboardInterrupt:
        _abort()
