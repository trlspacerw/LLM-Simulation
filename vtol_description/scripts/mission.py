#!/usr/bin/env python3
"""
mission.py — Automatic VTOL takeoff → hover → land mission.

Connects directly to Gazebo via gz.transport (no MAVLink, no ROS topics).
The model climbs to a configurable altitude, hovers, then lands — fully
automatic, no key-presses required.

Anti-flicker design
───────────────────
A background *pose-streaming thread* runs at STREAM_HZ (60 Hz) and is the
ONLY code that calls set_pose.  The motion ramp loop only writes to a shared
_target_z float — it never calls set_pose itself.  This means:

  • Service latency never accumulates into the ramp timer.  If one set_pose
    call takes 70 ms instead of 16 ms, the next call fires immediately (the
    ramp has already moved on to the correct z), so the model catches up in
    one frame instead of stuttering for multiple frames.

  • The model is kept pinned to GROUND_Z while on the ground (the streaming
    thread keeps running even when idle), which prevents the tiny z-drifts
    that cause ground-plane z-fighting flicker.

  • Each service call uses a short timeout (80 ms).  A slow service call is
    abandoned instead of blocking the thread, so the stream rate stays
    consistently close to 60 Hz.

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
"""

import argparse
import math
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

# ── shared state (ramp thread writes; pose thread reads) ──────────────────────
_target_z  = GROUND_Z      # Gazebo z that the pose thread streams
_z_lock    = threading.Lock()
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


def _make_pose_msg(z: float) -> pose_pb2.Pose:
    p = pose_pb2.Pose()
    p.name          = MODEL
    p.position.z    = float(z)
    p.orientation.w = _QUAT[0]
    p.orientation.x = _QUAT[1]
    p.orientation.y = _QUAT[2]
    p.orientation.z = _QUAT[3]
    return p


def _set_pose_once(z: float, timeout_ms: int = 400) -> bool:
    ok, _ = _node.request(
        SERVICE, _make_pose_msg(z),
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
            _set_pose_once(GROUND_Z, timeout_ms=200)
            time.sleep(0.1)     # 10 Hz is plenty while idle
            continue

        t0 = time.monotonic()
        with _z_lock:
            z = _target_z

        # Short timeout: abandon slow calls instantly so the stream stays timely
        _set_pose_once(z, timeout_ms=STREAM_TIMEOUT_MS)

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
    global _target_z
    dist     = abs(z_end - z_start)
    duration = max(0.5, dist / speed_mps)
    steps    = max(1, round(duration * STREAM_HZ))
    t0       = time.monotonic()

    for i in range(steps + 1):
        alpha = i / steps
        z     = z_start + alpha * (z_end - z_start)
        with _z_lock:
            _target_z = z
        agl = z - GROUND_Z
        elapsed = time.monotonic() - t0
        print(f'\r  [{label}]  z={z:.3f} m   AGL={agl:.2f} m   '
              f'({elapsed:.1f} / {duration:.1f} s)', end='', flush=True)
        gap = t0 + (i + 1) / STREAM_HZ - time.monotonic()
        if gap > 0:
            time.sleep(gap)

    # Hard-set final position to avoid rounding error
    with _z_lock:
        _target_z = z_end
    print()


# ── main mission ───────────────────────────────────────────────────────────────

def run(alt_m: float, hover_s: float, speed_mps: float):
    global _streaming, _target_z

    hover_z = GROUND_Z + alt_m

    # ── init gz transport ──────────────────────────────────────────────────────
    print('[mission] Initialising gz.transport ...', flush=True)
    _init_props()
    time.sleep(0.3)

    # ── wait for Gazebo set_pose service ─────────────────────────────────────
    print('[mission] Waiting for Gazebo (vtol_world) ...', flush=True)
    for attempt in range(40):
        if _set_pose_once(GROUND_Z):
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

    with _z_lock:
        _target_z = GROUND_Z

    # ── start background pose thread ──────────────────────────────────────────
    threading.Thread(target=_pose_stream_thread, daemon=True).start()

    # ── print mission summary ─────────────────────────────────────────────────
    print()
    print('┌─────────────────────────────────────────────────────┐')
    print('│              VTOL MISSION PARAMETERS                 │')
    print('├─────────────────────────────────────────────────────┤')
    print(f'│  Takeoff altitude : {alt_m:5.1f} m AGL                      │')
    print(f'│  Hover duration   : {hover_s:5.1f} s                         │')
    print(f'│  Climb/land speed : {speed_mps:5.1f} m/s                      │')
    print(f'│  Pose stream rate : {STREAM_HZ} Hz (anti-flicker thread)    │')
    print('└─────────────────────────────────────────────────────┘')
    print()
    print('  Ctrl+C at any time to abort (model returns to ground).')
    print()

    # ── ARM ────────────────────────────────────────────────────────────────────
    print('─' * 55)
    print('[mission]  PHASE 1/4 — ARM  (spooling up propellers)')
    print('─' * 55)
    _streaming = True          # pose thread starts streaming at 10 Hz (idle rate)
    _spool(+1, SPOOL_UP_S, 'spool-up')
    _set_props(1.0)
    print('[mission]  ✓ Armed\n')

    # ── TAKE-OFF ───────────────────────────────────────────────────────────────
    print('─' * 55)
    print(f'[mission]  PHASE 2/4 — TAKEOFF  → {alt_m:.1f} m AGL')
    print('─' * 55)
    _ramp('takeoff', GROUND_Z, hover_z, speed_mps)
    print(f'[mission]  ✓ Reached {alt_m:.1f} m AGL\n')

    # ── HOVER ──────────────────────────────────────────────────────────────────
    print('─' * 55)
    print(f'[mission]  PHASE 3/4 — HOVER  ({hover_s:.0f} s)')
    print('─' * 55)
    t_end = time.monotonic() + hover_s
    while time.monotonic() < t_end:
        rem = t_end - time.monotonic()
        with _z_lock:
            z = _target_z
        print(f'\r  [hover]  z={z:.3f} m   AGL={alt_m:.2f} m   '
              f'{rem:.0f} s remaining   ', end='', flush=True)
        time.sleep(0.2)
    print()
    print('[mission]  ✓ Hover complete\n')

    # ── LAND ───────────────────────────────────────────────────────────────────
    print('─' * 55)
    print('[mission]  PHASE 4/4 — LAND')
    print('─' * 55)
    _ramp('land', hover_z, GROUND_Z, speed_mps)
    with _z_lock:
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
    _set_pose_once(GROUND_Z, timeout_ms=1000)
    print('[mission]  Safe.', flush=True)


# ── entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Automatic VTOL mission: takeoff → hover → land',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Examples:\n'
            '  python3 mission.py                      # 5 m, 10 s hover\n'
            '  python3 mission.py --alt 15             # climb to 15 m\n'
            '  python3 mission.py --alt 10 --hover 30  # 10 m, 30 s hover\n'
            '  python3 mission.py --alt 8 --speed 3.0  # fast climb\n'
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
    args = parser.parse_args()

    if args.alt <= 0:
        parser.error('--alt must be > 0')
    if args.hover <= 0:
        parser.error('--hover must be > 0')
    if args.speed <= 0:
        parser.error('--speed must be > 0')

    try:
        run(args.alt, args.hover, args.speed)
    except KeyboardInterrupt:
        _abort()
