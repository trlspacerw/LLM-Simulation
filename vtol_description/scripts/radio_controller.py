#!/usr/bin/env python3
"""
radio_controller.py — Manual VTOL control via RadioMaster GX12 joystick.

Reads the RadioMaster controller (Mode 2 layout) and drives the VTOL in
Gazebo via gz.transport. Supports two modes:

  Physics (default): 6-DOF rigid body simulation with gravity, drag, wind.
  Kinematic (--kinematic): Direct position control (legacy behavior).

Stick Mapping (Mode 2)
──────────────────────
  Axis 0  Right X   Aileron    → roll (physics) / lateral strafe (kinematic)
  Axis 1  Right Y   Elevator   → pitch (physics) / forward/back (kinematic)
  Axis 2  Left  Y   Throttle   → collective thrust (physics) / climb (kinematic)
  Axis 3  Left  X   Rudder     → yaw rotation

Controls
────────
  Throttle centre = hover (physics mode); up = climb; down = descend
  Button 0 (or axis 4 switch) toggles arm/disarm
  Button 1 (or --pusher-button N) toggles pusher motor (forward thrust)
  Deadzone: 0.05 on all sticks

Usage
─────
  # Gazebo must already be running:
  ros2 launch vtol_description gazebo.launch.py

  # Physics mode (default):
  python3 radio_controller.py
  python3 radio_controller.py --wind 3.0 --gust 1.5

  # Kinematic mode (legacy):
  python3 radio_controller.py --kinematic
"""

import argparse
import math
import os
import sys
import threading
import time

try:
    import pygame
except ImportError:
    print(
        '[radio] ERROR: pygame is required.\n'
        '  Install it with:  pip install pygame',
        file=sys.stderr,
    )
    sys.exit(1)

import numpy as np

import gz.transport13    as transport
import gz.msgs10.pose_pb2    as pose_pb2
import gz.msgs10.boolean_pb2 as boolean_pb2
import gz.msgs10.double_pb2  as double_pb2

# Import physics engine
from vtol_physics import VTOLPhysics, MODE_QUAD, MODE_TRANSITION, MODE_PLANE

# ── world / model constants ─────────────────────────────────────────────────
WORLD   = 'vtol_world'
MODEL   = 'vtol'
SERVICE = f'/world/{WORLD}/set_pose'

GROUND_Z = 0.333
MAX_ALT  = 50.0

_h = math.sqrt(2) / 2
_QUAT = (float(_h), float(_h), 0.0, 0.0)  # (w, x, y, z) — base orientation

# ── timing constants ────────────────────────────────────────────────────────
STREAM_HZ         = 60
STREAM_TIMEOUT_MS = 80
SPOOL_UP_S        = 1.5
SPOOL_DOWN_S      = 1.5
CONTROL_HZ        = 50

# ── joystick axes ───────────────────────────────────────────────────────────
AXIS_AILERON  = 0   # Right X → roll / lateral
AXIS_ELEVATOR = 1   # Right Y → pitch / forward
AXIS_THROTTLE = 2   # Left Y  → throttle / climb
AXIS_RUDDER   = 3   # Left X  → yaw
AXIS_ARM_SW   = 4   # Switch axis used as alternate arm toggle
DEADZONE      = 0.05
BTN_ARM       = 0   # Button 0 toggles arm/disarm
BTN_PUSHER    = 1   # Button 1 toggles pusher (default)

PROP_JOINTS = [
    ('prop_1_joint', +50.0),  # FL (CCW)
    ('prop_2_joint', -50.0),  # FR (CW)
    ('prop_3_joint', +50.0),  # BL (CW)
    ('prop_4_joint', -50.0),  # BR (CCW)
    ('pusher_joint', +30.0),
]

SURFACE_JOINTS = [
    'aileron_link_joint',
    'ruddervator_left_link_joint',
    'ruddervator_right_link_joint',
]
SURFACE_MAX_DEG = 30.0  # max deflection in degrees

BTN_TRANSITION = 2  # Button 2 toggles quad/plane transition

# ── obstacle avoidance ──────────────────────────────────────────────────────
HALT_FILE = '/tmp/vtol_obstacle_halt'
_avoidance_enabled = False
_last_halt_state   = False


def _check_halt() -> bool:
    if not _avoidance_enabled:
        return False
    try:
        with open(HALT_FILE, 'r') as f:
            return f.read().strip() == '1'
    except OSError:
        return False


# ── shared state ────────────────────────────────────────────────────────────
_target_x  = 0.0
_target_y  = 0.0
_target_z  = GROUND_Z
_target_quat = _QUAT  # (w, x, y, z)
_pose_lock = threading.Lock()
_streaming = False
_node      = transport.Node()
_prop_pubs: dict = {}
_surface_pubs: dict = {}


# ── gz helpers ──────────────────────────────────────────────────────────────

def _init_props():
    for name, _ in PROP_JOINTS:
        topic = f'/model/{MODEL}/joint/{name}/cmd_vel'
        _prop_pubs[name] = _node.advertise(topic, double_pb2.Double)
    for name in SURFACE_JOINTS:
        topic = f'/model/{MODEL}/joint/{name}/0/cmd_pos'
        _surface_pubs[name] = _node.advertise(topic, double_pb2.Double)


def _set_props(scale: float):
    """Set all props to uniform scale (used for spool up/down)."""
    msg = double_pb2.Double()
    for name, max_vel in PROP_JOINTS:
        msg.data = max_vel * float(scale)
        _prop_pubs[name].publish(msg)


def _set_props_individual(rotor_speeds, pusher_speed):
    """Set individual rotor velocities from physics engine."""
    msg = double_pb2.Double()
    for i, (name, max_vel) in enumerate(PROP_JOINTS):
        if i < 4:
            msg.data = max_vel * float(rotor_speeds[i])
        else:
            msg.data = max_vel * float(pusher_speed)
        _prop_pubs[name].publish(msg)


def _set_surfaces(aileron, elevator, rudder):
    """Set control surface positions (aileron, ruddervators).

    Args:
        aileron: [-1, 1] aileron deflection
        elevator: [-1, 1] elevator (symmetric ruddervator)
        rudder: [-1, 1] rudder (differential ruddervator)
    """
    max_rad = math.radians(SURFACE_MAX_DEG)
    msg = double_pb2.Double()

    # Aileron
    msg.data = float(aileron) * max_rad
    _surface_pubs['aileron_link_joint'].publish(msg)

    # Ruddervators: left = elevator + rudder, right = elevator - rudder
    left = max(-1.0, min(1.0, elevator + rudder))
    right = max(-1.0, min(1.0, elevator - rudder))
    msg.data = float(left) * max_rad
    _surface_pubs['ruddervator_left_link_joint'].publish(msg)
    msg.data = float(right) * max_rad
    _surface_pubs['ruddervator_right_link_joint'].publish(msg)


def _make_pose_msg(x, y, z, quat=None):
    if quat is None:
        quat = _QUAT
    p = pose_pb2.Pose()
    p.name          = MODEL
    p.position.x    = float(x)
    p.position.y    = float(y)
    p.position.z    = float(z)
    p.orientation.w = float(quat[0])
    p.orientation.x = float(quat[1])
    p.orientation.y = float(quat[2])
    p.orientation.z = float(quat[3])
    return p


def _set_pose_once(x, y, z, quat=None, timeout_ms=400):
    ok, _ = _node.request(
        SERVICE, _make_pose_msg(x, y, z, quat),
        pose_pb2.Pose, boolean_pb2.Boolean, timeout_ms
    )
    return ok


# ── pose-streaming thread ──────────────────────────────────────────────────

def _pose_stream_thread():
    dt = 1.0 / STREAM_HZ
    while True:
        if not _streaming:
            with _pose_lock:
                x, y = _target_x, _target_y
                quat = _target_quat
            _set_pose_once(x, y, GROUND_Z, quat=quat, timeout_ms=200)
            time.sleep(0.1)
            continue

        t0 = time.monotonic()
        with _pose_lock:
            x, y, z = _target_x, _target_y, _target_z
            quat = _target_quat

        _set_pose_once(x, y, z, quat=quat, timeout_ms=STREAM_TIMEOUT_MS)

        elapsed = time.monotonic() - t0
        gap = dt - elapsed
        if gap > 0:
            time.sleep(gap)


# ── prop spool ramp ─────────────────────────────────────────────────────────

def _spool(direction: int, duration: float, label: str):
    steps = max(1, round(duration * STREAM_HZ))
    t0 = time.monotonic()
    for i in range(steps + 1):
        alpha = i / steps
        scale = alpha if direction > 0 else 1.0 - alpha
        _set_props(scale)
        bar = '\u2588' * int(scale * 24) + '\u2591' * (24 - int(scale * 24))
        print(f'\r  [{label}]  [{bar}]  {scale*100:3.0f}%', end='', flush=True)
        gap = t0 + (i + 1) / STREAM_HZ - time.monotonic()
        if gap > 0:
            time.sleep(gap)
    print()


# ── stick helpers ───────────────────────────────────────────────────────────

def _apply_deadzone(value: float, dz: float = DEADZONE) -> float:
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)


# ── main control loop (PHYSICS mode) ─────────────────────────────────────────

def run_physics(max_speed, max_climb, wind_speed, wind_dir, gust, pusher_button, transition_button):
    global _streaming, _target_x, _target_y, _target_z, _target_quat, _last_halt_state

    # ── init physics ─────────────────────────────────────────────────────────
    physics = VTOLPhysics()
    physics.position = np.array([0.0, 0.0, GROUND_Z])

    # Wind setup
    if wind_speed > 0:
        wind_rad = math.radians(wind_dir)
        physics.wind_velocity = np.array([
            wind_speed * math.cos(wind_rad),
            wind_speed * math.sin(wind_rad),
            0.0
        ])
    if gust > 0:
        physics.gust_magnitude = gust

    # ── init pygame joystick ─────────────────────────────────────────────────
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print(
            '[radio] ERROR: No joystick found.\n'
            '  Make sure the RadioMaster GX12 is connected via USB.\n'
            '  Check with: ls /dev/input/js*',
            file=sys.stderr,
        )
        pygame.quit()
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f'[radio] Joystick: {joy.get_name()}')
    print(f'[radio] Axes: {joy.get_numaxes()}  Buttons: {joy.get_numbuttons()}')

    # ── init gz transport ────────────────────────────────────────────────────
    print('[radio] Initialising gz.transport ...', flush=True)
    _init_props()
    time.sleep(0.3)

    print('[radio] Waiting for Gazebo (vtol_world) ...', flush=True)
    for attempt in range(40):
        if _set_pose_once(0.0, 0.0, GROUND_Z):
            break
        if attempt % 4 == 0:
            print(f'  (attempt {attempt + 1}/40 — is Gazebo running?)', flush=True)
        time.sleep(0.5)
    else:
        print(
            '\n[radio] ERROR: cannot reach /world/vtol_world/set_pose\n'
            '  Start Gazebo first:\n'
            '    source ~/ardu_ws/install/setup.bash\n'
            '    ros2 launch vtol_description gazebo.launch.py',
            file=sys.stderr,
        )
        pygame.quit()
        sys.exit(1)

    with _pose_lock:
        _target_x = 0.0
        _target_y = 0.0
        _target_z = GROUND_Z
        _target_quat = _QUAT

    # ── start background pose thread ─────────────────────────────────────────
    threading.Thread(target=_pose_stream_thread, daemon=True).start()

    # ── print summary ────────────────────────────────────────────────────────
    print()
    print('┌─────────────────────────────────────────────────────┐')
    print('│       RADIO CONTROLLER  (Physics Mode 2)            │')
    print('├─────────────────────────────────────────────────────┤')
    print(f'│  Mass            : {physics.mass:.1f} kg                           │')
    print(f'│  Hover throttle  : 50%                              │')
    print(f'│  Max altitude    : {MAX_ALT:.0f} m AGL                        │')
    if wind_speed > 0:
        print(f'│  Wind            : {wind_speed:.1f} m/s from {wind_dir:.0f}°              │')
    if gust > 0:
        print(f'│  Gusts           : up to {gust:.1f} m/s                    │')
    if _avoidance_enabled:
        print(f'│  Obstacle avoid. : ENABLED                          │')
    print('├─────────────────────────────────────────────────────┤')
    print('│  Right Stick X : Roll / Aileron                      │')
    print('│  Right Stick Y : Pitch / Elevator                    │')
    print('│  Left  Stick Y : Throttle (collective / pusher)      │')
    print('│  Left  Stick X : Yaw / Rudder                        │')
    print(f'│  Button {transition_button}       : Transition (QUAD↔PLANE)          │')
    print(f'│  Button {pusher_button}       : Toggle pusher motor              │')
    print('│  Button 0      : Arm / Disarm toggle                │')
    print('│  Ctrl+C        : Abort and land                     │')
    print('└─────────────────────────────────────────────────────┘')
    print()

    # ── state ────────────────────────────────────────────────────────────────
    armed = False
    prev_btn_arm = 0
    prev_sw_arm  = -1.0
    prev_btn_pusher = 0
    prev_btn_transition = 0
    pusher_active = False
    dt = 1.0 / CONTROL_HZ

    print('[radio] Waiting for ARM (press Button 0) ...', flush=True)

    while True:
        t0 = time.monotonic()
        pygame.event.pump()

        # ── read sticks ──────────────────────────────────────────────────────
        aileron  = _apply_deadzone(joy.get_axis(AXIS_AILERON))
        elevator = _apply_deadzone(joy.get_axis(AXIS_ELEVATOR))
        throttle_raw = _apply_deadzone(joy.get_axis(AXIS_THROTTLE))
        rudder   = _apply_deadzone(joy.get_axis(AXIS_RUDDER))

        # Invert throttle: stick up (negative raw) = more thrust
        throttle_raw = -throttle_raw

        # Map throttle from [-1,1] to [0,1]: center=0.5 (hover)
        throttle = (throttle_raw + 1.0) * 0.5

        # ── arm/disarm toggle via button 0 (rising edge) ─────────────────────
        btn_arm = joy.get_button(BTN_ARM)
        if btn_arm and not prev_btn_arm:
            armed = not armed
            if armed:
                print('\n[radio] ARMING ...', flush=True)
                _streaming = True
                physics.reset(position=[_target_x, _target_y, GROUND_Z])
                _spool(+1, SPOOL_UP_S, 'spool-up')
                print('[radio] ARMED — 50% throttle = hover', flush=True)
            else:
                print('\n[radio] DISARMING ...', flush=True)
                _spool(-1, SPOOL_DOWN_S, 'spool-down')
                _set_props(0.0)
                _streaming = False
                pusher_active = False
                with _pose_lock:
                    _target_z = GROUND_Z
                    _target_quat = _QUAT
                print('[radio] DISARMED', flush=True)
        prev_btn_arm = btn_arm

        # ── arm/disarm toggle via switch axis 4 (threshold) ──────────────────
        if joy.get_numaxes() > AXIS_ARM_SW:
            sw = joy.get_axis(AXIS_ARM_SW)
            sw_active = sw > 0.5
            prev_active = prev_sw_arm > 0.5
            if sw_active and not prev_active:
                armed = not armed
                if armed:
                    print('\n[radio] ARMING (switch) ...', flush=True)
                    _streaming = True
                    physics.reset(position=[_target_x, _target_y, GROUND_Z])
                    _spool(+1, SPOOL_UP_S, 'spool-up')
                    print('[radio] ARMED — 50% throttle = hover', flush=True)
                else:
                    print('\n[radio] DISARMING (switch) ...', flush=True)
                    _spool(-1, SPOOL_DOWN_S, 'spool-down')
                    _set_props(0.0)
                    _streaming = False
                    pusher_active = False
                    with _pose_lock:
                        _target_z = GROUND_Z
                        _target_quat = _QUAT
                    print('[radio] DISARMED', flush=True)
            prev_sw_arm = sw

        # ── pusher toggle ────────────────────────────────────────────────────
        if joy.get_numbuttons() > pusher_button:
            btn_p = joy.get_button(pusher_button)
            if btn_p and not prev_btn_pusher:
                pusher_active = not pusher_active
                state = 'ON' if pusher_active else 'OFF'
                print(f'\n[radio] Pusher: {state}', flush=True)
            prev_btn_pusher = btn_p

        # ── transition button ──────────────────────────────────────────────
        if armed and joy.get_numbuttons() > transition_button:
            btn_t = joy.get_button(transition_button)
            if btn_t and not prev_btn_transition:
                prev_mode = physics.mode
                physics.start_transition()
                if physics.mode == MODE_TRANSITION:
                    if physics.transition_direction > 0:
                        print(f'\n[radio] TRANSITION: QUAD → PLANE', flush=True)
                    else:
                        print(f'\n[radio] TRANSITION: PLANE → QUAD', flush=True)
            prev_btn_transition = btn_t

        # ── update physics if armed ──────────────────────────────────────────
        if armed:
            halted = _check_halt()
            if halted:
                if not _last_halt_state:
                    print('\n  [HALT] Obstacle detected — holding position', flush=True)
                    _last_halt_state = True
                # Zero inputs when halted
                physics.set_inputs(0.5, 0.0, 0.0, 0.0, 0.0)
            else:
                if _last_halt_state:
                    print('\n  [CLEAR] Obstacle cleared', flush=True)
                    _last_halt_state = False

                pusher_val = 1.0 if pusher_active else 0.0
                physics.set_inputs(throttle, aileron, -elevator, rudder, pusher_val)

            physics.step(dt)

            # Update shared pose from physics
            pos = physics.position
            gz_quat = physics.get_gazebo_quaternion()

            with _pose_lock:
                _target_x = pos[0]
                _target_y = pos[1]
                _target_z = pos[2]
                _target_quat = (gz_quat[0], gz_quat[1], gz_quat[2], gz_quat[3])

            # Set individual prop speeds
            _set_props_individual(physics.rotor_speeds, physics.pusher_speed)

            # Animate control surfaces
            _set_surfaces(
                physics.aileron_deflection,
                physics.elevator_deflection,
                physics.rudder_deflection,
            )

        # ── HUD ──────────────────────────────────────────────────────────────
        with _pose_lock:
            x, y, z = _target_x, _target_y, _target_z
        agl = z - GROUND_Z
        arm_str = 'ARMED' if armed else 'DISARMED'
        halt_str = ' [HALT]' if (_last_halt_state and armed) else ''

        if armed:
            roll_d, pitch_d, yaw_d = physics.get_euler()
            airspeed = physics.get_airspeed()

            # Flight mode indicator
            mode = physics.mode
            if mode == MODE_QUAD:
                mode_str = 'QUAD'
            elif mode == MODE_PLANE:
                mode_str = 'PLANE'
            elif physics.transition_direction > 0:
                mode_str = 'TRANS\u2192FW'
            else:
                mode_str = 'TRANS\u2192MC'

            # Airspeed bar during transition
            trans_bar = ''
            if mode == MODE_TRANSITION:
                if physics.transition_direction > 0:
                    progress = min(1.0, airspeed / physics.transition_speed)
                else:
                    progress = min(1.0, 1.0 - airspeed / physics.transition_speed) if physics.transition_speed > 0 else 0
                bar_len = 10
                filled = int(progress * bar_len)
                bar_str = '\u2588' * filled + '\u2591' * (bar_len - filled)
                trans_bar = f' [{bar_str}]'

            print(
                f'\r  [{arm_str}] [{mode_str}]  '
                f'Thr={throttle*100:3.0f}%  '
                f'R={roll_d:+5.1f}\u00b0 P={pitch_d:+5.1f}\u00b0 Y={yaw_d:+5.1f}\u00b0  '
                f'AS={airspeed:.1f}m/s  AGL={agl:.1f}m'
                f'{trans_bar}{halt_str}     ',
                end='', flush=True,
            )
        else:
            print(
                f'\r  [{arm_str}]  '
                f'Ail={aileron:+.2f}  Ele={elevator:+.2f}  Thr={throttle_raw:+.2f}  Rud={rudder:+.2f}'
                f'     ',
                end='', flush=True,
            )

        # ── timing ───────────────────────────────────────────────────────────
        elapsed = time.monotonic() - t0
        gap = dt - elapsed
        if gap > 0:
            time.sleep(gap)


# ── main control loop (KINEMATIC mode — legacy) ──────────────────────────────

def run_kinematic(max_speed, max_climb):
    global _streaming, _target_x, _target_y, _target_z, _target_quat, _last_halt_state

    # ── init pygame joystick ─────────────────────────────────────────────────
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print(
            '[radio] ERROR: No joystick found.\n'
            '  Make sure the RadioMaster GX12 is connected via USB.\n'
            '  Check with: ls /dev/input/js*',
            file=sys.stderr,
        )
        pygame.quit()
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f'[radio] Joystick: {joy.get_name()}')
    print(f'[radio] Axes: {joy.get_numaxes()}  Buttons: {joy.get_numbuttons()}')

    # ── init gz transport ────────────────────────────────────────────────────
    print('[radio] Initialising gz.transport ...', flush=True)
    _init_props()
    time.sleep(0.3)

    print('[radio] Waiting for Gazebo (vtol_world) ...', flush=True)
    for attempt in range(40):
        if _set_pose_once(0.0, 0.0, GROUND_Z):
            break
        if attempt % 4 == 0:
            print(f'  (attempt {attempt + 1}/40 — is Gazebo running?)', flush=True)
        time.sleep(0.5)
    else:
        print(
            '\n[radio] ERROR: cannot reach /world/vtol_world/set_pose\n'
            '  Start Gazebo first:\n'
            '    source ~/ardu_ws/install/setup.bash\n'
            '    ros2 launch vtol_description gazebo.launch.py',
            file=sys.stderr,
        )
        pygame.quit()
        sys.exit(1)

    with _pose_lock:
        _target_x = 0.0
        _target_y = 0.0
        _target_z = GROUND_Z
        _target_quat = _QUAT

    # ── start background pose thread ─────────────────────────────────────────
    threading.Thread(target=_pose_stream_thread, daemon=True).start()

    # ── print summary ────────────────────────────────────────────────────────
    print()
    print('┌─────────────────────────────────────────────────────┐')
    print('│       RADIO CONTROLLER  (Kinematic Mode 2)          │')
    print('├─────────────────────────────────────────────────────┤')
    print(f'│  Max lateral speed : {max_speed:5.1f} m/s                     │')
    print(f'│  Max climb speed   : {max_climb:5.1f} m/s                     │')
    print(f'│  Altitude range    : 0 — {MAX_ALT:.0f} m AGL                  │')
    print(f'│  Pose stream rate  : {STREAM_HZ} Hz                          │')
    if _avoidance_enabled:
        print(f'│  Obstacle avoid.   : ENABLED                         │')
    print('├─────────────────────────────────────────────────────┤')
    print('│  Right Stick X : Strafe left/right                  │')
    print('│  Right Stick Y : Forward/backward                   │')
    print('│  Left  Stick Y : Throttle (climb/descend)           │')
    print('│  Left  Stick X : Yaw (rotate)                       │')
    print('│  Button 0      : Arm / Disarm toggle                │')
    print('│  Ctrl+C        : Abort and land                     │')
    print('└─────────────────────────────────────────────────────┘')
    print()

    # ── state ────────────────────────────────────────────────────────────────
    armed = False
    prev_btn_arm = 0
    prev_sw_arm  = -1.0
    yaw_angle = 0.0  # accumulated yaw in radians
    dt = 1.0 / CONTROL_HZ

    print('[radio] Waiting for ARM (press Button 0) ...', flush=True)

    while True:
        t0 = time.monotonic()
        pygame.event.pump()

        # ── read sticks ──────────────────────────────────────────────────────
        aileron  = _apply_deadzone(joy.get_axis(AXIS_AILERON))
        elevator = _apply_deadzone(joy.get_axis(AXIS_ELEVATOR))
        throttle = _apply_deadzone(joy.get_axis(AXIS_THROTTLE))
        rudder   = _apply_deadzone(joy.get_axis(AXIS_RUDDER))
        # Invert throttle: stick up (negative raw) = climb (positive)
        throttle = -throttle

        # ── arm/disarm toggle via button 0 (rising edge) ─────────────────────
        btn_arm = joy.get_button(BTN_ARM)
        if btn_arm and not prev_btn_arm:
            armed = not armed
            if armed:
                print('\n[radio] ARMING ...', flush=True)
                _streaming = True
                _spool(+1, SPOOL_UP_S, 'spool-up')
                _set_props(1.0)
                print('[radio] ARMED', flush=True)
            else:
                print('\n[radio] DISARMING ...', flush=True)
                _spool(-1, SPOOL_DOWN_S, 'spool-down')
                _set_props(0.0)
                _streaming = False
                with _pose_lock:
                    _target_z = GROUND_Z
                    _target_quat = _QUAT
                yaw_angle = 0.0
                print('[radio] DISARMED', flush=True)
        prev_btn_arm = btn_arm

        # ── arm/disarm toggle via switch axis 4 (threshold) ──────────────────
        if joy.get_numaxes() > AXIS_ARM_SW:
            sw = joy.get_axis(AXIS_ARM_SW)
            sw_active = sw > 0.5
            prev_active = prev_sw_arm > 0.5
            if sw_active and not prev_active:
                armed = not armed
                if armed:
                    print('\n[radio] ARMING (switch) ...', flush=True)
                    _streaming = True
                    _spool(+1, SPOOL_UP_S, 'spool-up')
                    _set_props(1.0)
                    print('[radio] ARMED', flush=True)
                else:
                    print('\n[radio] DISARMING (switch) ...', flush=True)
                    _spool(-1, SPOOL_DOWN_S, 'spool-down')
                    _set_props(0.0)
                    _streaming = False
                    with _pose_lock:
                        _target_z = GROUND_Z
                        _target_quat = _QUAT
                    yaw_angle = 0.0
                    print('[radio] DISARMED', flush=True)
            prev_sw_arm = sw

        # ── update target position if armed ──────────────────────────────────
        if armed:
            halted = _check_halt()
            if halted:
                if not _last_halt_state:
                    print('\n  [HALT] Obstacle detected — holding position', flush=True)
                    _last_halt_state = True
            else:
                if _last_halt_state:
                    print('\n  [CLEAR] Obstacle cleared', flush=True)
                    _last_halt_state = False

                # Yaw accumulation
                yaw_rate = 2.0  # rad/s at full deflection
                yaw_angle += rudder * yaw_rate * dt

                with _pose_lock:
                    # Movement in body frame, rotated by yaw
                    cos_y = math.cos(yaw_angle)
                    sin_y = math.sin(yaw_angle)
                    dx = aileron * max_speed * dt
                    dy = elevator * max_speed * dt
                    _target_x += dx * cos_y - dy * sin_y
                    _target_y += dx * sin_y + dy * cos_y
                    _target_z += throttle * max_climb * dt
                    _target_z = max(GROUND_Z, min(GROUND_Z + MAX_ALT, _target_z))

                    # Compose base rotation with yaw
                    # Base: 90° roll around X. Then yaw around world Z.
                    from vtol_physics import quat_from_euler, quat_multiply as qm
                    q_base = np.array([_h, _h, 0.0, 0.0])
                    q_yaw = quat_from_euler(0.0, 0.0, yaw_angle)
                    q_final = qm(q_yaw, q_base)
                    _target_quat = (q_final[0], q_final[1], q_final[2], q_final[3])

        # ── HUD ──────────────────────────────────────────────────────────────
        with _pose_lock:
            x, y, z = _target_x, _target_y, _target_z
        agl = z - GROUND_Z
        arm_str = 'ARMED' if armed else 'DISARMED'
        halt_str = ' [HALT]' if (_last_halt_state and armed) else ''
        yaw_deg = math.degrees(yaw_angle)
        print(
            f'\r  [{arm_str}]  '
            f'Ail={aileron:+.2f}  Ele={elevator:+.2f}  Thr={throttle:+.2f}  Rud={rudder:+.2f}  '
            f'Y={yaw_deg:+.0f}°  AGL={agl:.1f}m'
            f'{halt_str}     ',
            end='', flush=True,
        )

        # ── timing ───────────────────────────────────────────────────────────
        elapsed = time.monotonic() - t0
        gap = dt - elapsed
        if gap > 0:
            time.sleep(gap)


# ── abort handler ───────────────────────────────────────────────────────────

def _abort():
    print('\n[radio] Abort — stopping props and returning to ground ...', flush=True)
    _set_props(0.0)
    with _pose_lock:
        x, y = _target_x, _target_y
    _set_pose_once(x, y, GROUND_Z, quat=_QUAT, timeout_ms=1000)
    print('[radio] Safe.', flush=True)
    pygame.quit()


# ── entry point ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Manual VTOL control via RadioMaster joystick (6-DOF physics)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Examples:\n'
            '  python3 radio_controller.py                        # physics mode\n'
            '  python3 radio_controller.py --wind 3.0 --gust 1.5  # with wind\n'
            '  python3 radio_controller.py --kinematic             # legacy mode\n'
        ),
    )
    parser.add_argument(
        '--kinematic', action='store_true',
        help='Use kinematic (direct position) control instead of physics',
    )
    parser.add_argument(
        '--max-speed', type=float, default=3.0, metavar='M_PER_S',
        help='Max lateral/forward speed in m/s (kinematic mode, default: 3.0)',
    )
    parser.add_argument(
        '--max-climb', type=float, default=2.0, metavar='M_PER_S',
        help='Max climb/descend speed in m/s (kinematic mode, default: 2.0)',
    )
    parser.add_argument(
        '--wind', type=float, default=0.0, metavar='M_PER_S',
        help='Constant wind speed in m/s (default: 0, physics mode only)',
    )
    parser.add_argument(
        '--wind-dir', type=float, default=0.0, metavar='DEG',
        help='Wind direction in degrees (0=+X, 90=+Y, default: 0)',
    )
    parser.add_argument(
        '--gust', type=float, default=0.0, metavar='M_PER_S',
        help='Random gust magnitude in m/s (default: 0, physics mode only)',
    )
    parser.add_argument(
        '--pusher-button', type=int, default=BTN_PUSHER, metavar='N',
        help=f'Button number for pusher toggle (default: {BTN_PUSHER})',
    )
    parser.add_argument(
        '--transition-button', type=int, default=BTN_TRANSITION, metavar='N',
        help=f'Button number for QUAD/PLANE transition (default: {BTN_TRANSITION})',
    )
    parser.add_argument(
        '--avoidance', action='store_true',
        help='Enable obstacle avoidance (reads /tmp/vtol_obstacle_halt)',
    )
    args = parser.parse_args()

    if args.max_speed <= 0:
        parser.error('--max-speed must be > 0')
    if args.max_climb <= 0:
        parser.error('--max-climb must be > 0')

    if args.avoidance:
        _avoidance_enabled = True
        print('[radio] Obstacle avoidance ENABLED — polling', HALT_FILE)

    try:
        if args.kinematic:
            run_kinematic(args.max_speed, args.max_climb)
        else:
            run_physics(args.max_speed, args.max_climb,
                       args.wind, args.wind_dir, args.gust,
                       args.pusher_button, args.transition_button)
    except KeyboardInterrupt:
        _abort()
