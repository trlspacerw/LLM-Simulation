#!/usr/bin/env python3
"""
DroneInterface — MAVLink arm/takeoff/Turn/Move commands for ArduPilot SITL.

Based on the MavController pattern from active_tracker.py.
"""

import time
import math
from pymavlink import mavutil


class DroneInterface:
    """MAVLink interface for arm, takeoff, Turn(θ), and Move(d) commands."""

    def __init__(self, connection_string='udpin:127.0.0.1:14550',
                 hover_alt_m=8.0):
        self._conn_str = connection_string
        self.hover_alt = hover_alt_m
        self.conn = None

    # ── connection ────────────────────────────────────────────────────────

    def connect(self):
        print(f'[MAV] Connecting to {self._conn_str} ...')
        self.conn = mavutil.mavlink_connection(self._conn_str)
        self.conn.wait_heartbeat()
        print(f'[MAV] Heartbeat OK  '
              f'(sys={self.conn.target_system} '
              f'comp={self.conn.target_component})')

    # ── mode / arm / takeoff ──────────────────────────────────────────────

    def set_mode(self, mode_name):
        mode_id = self.conn.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f'[MAV] Unknown mode {mode_name}')
            return
        self.conn.set_mode(mode_id)
        print(f'[MAV] Mode → {mode_name}')

    def arm(self):
        print('[MAV] Arming ...', flush=True)
        for attempt in range(5):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
            deadline = time.time() + 3
            while time.time() < deadline:
                msg = self.conn.recv_match(
                    type=['COMMAND_ACK', 'HEARTBEAT'],
                    blocking=True, timeout=1)
                if msg is None:
                    continue
                if msg.get_type() == 'HEARTBEAT':
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        print('[MAV] Armed', flush=True)
                        return True
                elif (msg.get_type() == 'COMMAND_ACK'
                      and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print('[MAV] Armed', flush=True)
                        return True
                    print(f'[MAV] Arm rejected (result={msg.result}), '
                          f'retry {attempt+1}/5', flush=True)
                    break
            time.sleep(1)

        # Force arm: param2=21196 bypasses all ArduPilot pre-arm checks (SITL)
        print('[MAV] Trying force arm (param2=21196) ...', flush=True)
        for _ in range(5):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)
            deadline = time.time() + 3
            while time.time() < deadline:
                msg = self.conn.recv_match(
                    type=['COMMAND_ACK', 'HEARTBEAT'],
                    blocking=True, timeout=1)
                if msg is None:
                    continue
                if msg.get_type() == 'HEARTBEAT':
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        print('[MAV] Force armed!', flush=True)
                        return True
                elif (msg.get_type() == 'COMMAND_ACK'
                      and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print('[MAV] Force armed!', flush=True)
                        return True
                    print(f'[MAV] Force arm rejected (result={msg.result})', flush=True)
                    break
            time.sleep(1)

        print('[MAV] Failed to arm (including force arm)', flush=True)
        return False

    def takeoff(self, alt=None):
        alt = alt or self.hover_alt
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt)
        print(f'[MAV] Takeoff to {alt} m')

    def _disable_arming_checks(self):
        """Set ARMING_CHECK=0 and DISARM_DELAY=0 for SITL.

        DISARM_DELAY=0 is critical: without it ArduCopter auto-disarms
        before NAV_TAKEOFF can execute (default delay is ~10s but fires
        much sooner with force-arm in some configurations).
        """
        print('[MAV] Setting ARMING_CHECK=0 and DISARM_DELAY=0 ...', flush=True)
        for param_name, val in [(b'ARMING_CHECK', 0), (b'DISARM_DELAY', 0)]:
            self.conn.mav.param_set_send(
                self.conn.target_system, self.conn.target_component,
                param_name, val,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        # Give ArduCopter time to apply the parameters
        time.sleep(3)
        print('[MAV] SITL params applied', flush=True)

    def _wait_for_ekf_using_gps(self, timeout=120):
        """Block until EKF is using GPS.

        Watches STATUSTEXT for 'is using GPS' (primary signal) and falls back
        to sustained GPS_RAW_INT fix_type >= 3 for 30 consecutive seconds
        in case the STATUSTEXT is missed during mavproxy reconnection.
        """
        print('[MAV] Waiting for EKF to use GPS ...', flush=True)
        gps_fix_seen = False
        gps_good_since = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.conn.recv_match(
                type=['GPS_RAW_INT', 'STATUSTEXT'],
                blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == 'GPS_RAW_INT':
                if msg.fix_type >= 3:
                    if not gps_fix_seen:
                        print(f'[MAV] GPS fix OK (fix_type={msg.fix_type}, '
                              f'sats={msg.satellites_visible})', flush=True)
                        gps_fix_seen = True
                        gps_good_since = time.time()
                    # Fallback: 30 consecutive seconds of good GPS = EKF ready
                    elif gps_good_since and (time.time() - gps_good_since) >= 30:
                        print('[MAV] EKF GPS ready (30s sustained fix)', flush=True)
                        time.sleep(3)
                        return True
                else:
                    gps_good_since = None
            elif msg.get_type() == 'STATUSTEXT':
                text = (msg.text.decode('utf-8', errors='replace')
                        if isinstance(msg.text, bytes) else str(msg.text))
                if 'using GPS' in text:
                    print('[MAV] EKF using GPS — ready to arm (3s stabilize)', flush=True)
                    time.sleep(3)
                    return True
        print('[MAV] EKF/GPS timeout — proceeding anyway', flush=True)
        return False

    def takeoff_and_hover(self):
        self.set_mode('GUIDED')
        time.sleep(2)
        print('[MAV] Waiting for EKF / pre-arm ...', flush=True)
        for _ in range(30):
            msg = self.conn.recv_match(type='HEARTBEAT',
                                       blocking=True, timeout=2)
            if msg and msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                print('[MAV] Vehicle ready', flush=True)
                break
            time.sleep(1)
        self._disable_arming_checks()
        # Wait for EKF to use GPS AFTER disabling arming checks so mavproxy
        # is settled before we start watching STATUSTEXT.
        self._wait_for_ekf_using_gps(timeout=180)

        # Force arm — retry until HEARTBEAT confirms ARMED.
        # Re-apply GUIDED mode each attempt in case it drifted.
        arm_confirmed = False
        for attempt in range(10):
            self.set_mode('GUIDED')
            time.sleep(0.5)
            print(f'[MAV] Force arming attempt {attempt+1} ...', flush=True)
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)
            deadline = time.time() + 5
            while time.time() < deadline:
                msg = self.conn.recv_match(
                    type=['HEARTBEAT', 'COMMAND_ACK', 'STATUSTEXT'],
                    blocking=True, timeout=1)
                if msg is None:
                    continue
                if msg.get_type() == 'COMMAND_ACK':
                    if (msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                            and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
                        print('[MAV] Force arm accepted', flush=True)
                elif msg.get_type() == 'HEARTBEAT':
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        arm_confirmed = True
                        print('[MAV] Armed (heartbeat confirmed)', flush=True)
                        break
                elif msg.get_type() == 'STATUSTEXT':
                    text = (msg.text.decode('utf-8', errors='replace')
                            if isinstance(msg.text, bytes) else str(msg.text))
                    print(f'[MAV] AP: {text.strip()}', flush=True)
            if arm_confirmed:
                break
            time.sleep(1)

        if not arm_confirmed:
            print('[MAV] WARNING: arm not confirmed after 10 attempts', flush=True)

        # NAV_TAKEOFF retry loop — also re-sets GUIDED mode and re-arms if
        # drone disarms, and watches STATUSTEXT for ArduPilot error messages.
        last_takeoff_time = 0
        last_arm_time = 0
        takeoff_accepted = False
        deadline = time.time() + 120
        while time.time() < deadline:
            msg = self.conn.recv_match(
                type=['GLOBAL_POSITION_INT', 'COMMAND_ACK', 'HEARTBEAT', 'STATUSTEXT'],
                blocking=False)
            if msg:
                t = msg.get_type()
                if t == 'GLOBAL_POSITION_INT':
                    alt_m = msg.relative_alt / 1000.0
                    if alt_m >= 1.0:
                        print(f'[MAV] Alt: {alt_m:.1f} m', flush=True)
                    if alt_m >= self.hover_alt * 0.85:
                        print(f'[MAV] Hover reached at {alt_m:.1f} m', flush=True)
                        return
                elif t == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print('[MAV] NAV_TAKEOFF accepted — climbing', flush=True)
                        takeoff_accepted = True
                    else:
                        print(f'[MAV] NAV_TAKEOFF result={msg.result}', flush=True)
                elif t == 'HEARTBEAT':
                    is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if not is_armed and arm_confirmed:
                        print('[MAV] Drone disarmed — re-arming ...', flush=True)
                        arm_confirmed = False
                        takeoff_accepted = False
                    elif is_armed:
                        arm_confirmed = True
                elif t == 'STATUSTEXT':
                    text = (msg.text.decode('utf-8', errors='replace')
                            if isinstance(msg.text, bytes) else str(msg.text))
                    print(f'[MAV] AP: {text.strip()}', flush=True)

            now = time.time()
            # Re-arm if needed
            if not arm_confirmed and (now - last_arm_time) > 3:
                self.set_mode('GUIDED')
                self.conn.mav.command_long_send(
                    self.conn.target_system, self.conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 21196, 0, 0, 0, 0, 0)
                last_arm_time = now
            # Send NAV_TAKEOFF while armed
            elif arm_confirmed and not takeoff_accepted and (now - last_takeoff_time) > 3:
                print(f'[MAV] NAV_TAKEOFF → {self.hover_alt}m', flush=True)
                self.conn.mav.command_long_send(
                    self.conn.target_system, self.conn.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, self.hover_alt)
                last_takeoff_time = now

            time.sleep(0.1)

        print('[MAV] Altitude timeout — continuing', flush=True)

    # ── low-level velocity command ────────────────────────────────────────

    def _send_body_velocity(self, vx, vy, vz, yaw_rate):
        """Send SET_POSITION_TARGET_LOCAL_NED in BODY_NED frame.

        vx: forward (m/s), vy: right (m/s), vz: down-positive (m/s),
        yaw_rate: clockwise-positive (rad/s).
        """
        # type_mask: ignore pos (0,1,2), accel (6,7,8), and yaw (10)
        type_mask = (1 << 0 | 1 << 1 | 1 << 2 |
                     1 << 6 | 1 << 7 | 1 << 8 |
                     1 << 10)
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,          # pos (ignored)
            vx, vy, vz,       # velocity
            0, 0, 0,          # accel (ignored)
            0, yaw_rate,      # yaw (ignored), yaw_rate
        )

    # ── discrete action primitives ────────────────────────────────────────

    def execute_turn(self, theta_deg, duration=2.0):
        """Rotate the drone by theta_deg (positive = clockwise) over duration seconds."""
        theta_rad = math.radians(theta_deg)
        yaw_rate = theta_rad / duration   # rad/s
        print(f'[MAV] Turn({theta_deg:.1f}°) @ {math.degrees(yaw_rate):.1f}°/s '
              f'for {duration:.1f}s')
        rate_hz = 10
        dt = 1.0 / rate_hz
        steps = int(duration * rate_hz)
        for _ in range(steps):
            self._send_body_velocity(0.0, 0.0, 0.0, yaw_rate)
            time.sleep(dt)
        # Hold (zero rate) to stop rotation
        self._send_body_velocity(0.0, 0.0, 0.0, 0.0)

    def execute_move(self, d_m, duration=2.0):
        """Move forward by d_m meters (negative = backward) over duration seconds."""
        vx = d_m / duration   # m/s
        print(f'[MAV] Move({d_m:.1f}m) @ {vx:.2f}m/s for {duration:.1f}s')
        rate_hz = 10
        dt = 1.0 / rate_hz
        steps = int(duration * rate_hz)
        for _ in range(steps):
            self._send_body_velocity(vx, 0.0, 0.0, 0.0)
            time.sleep(dt)
        # Stop
        self._send_body_velocity(0.0, 0.0, 0.0, 0.0)

    def hold_position(self):
        """Send zero velocity to hold current position."""
        self._send_body_velocity(0.0, 0.0, 0.0, 0.0)

    def return_to_launch(self, timeout=120):
        """Command RTL and wait until the drone lands and disarms."""
        print('[MAV] RTL — returning to launch position ...', flush=True)
        self.set_mode('RTL')
        landed = False
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.conn.recv_match(
                type=['GLOBAL_POSITION_INT', 'HEARTBEAT', 'STATUSTEXT'],
                blocking=True, timeout=2)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'GLOBAL_POSITION_INT':
                alt_m = msg.relative_alt / 1000.0
                if alt_m < 0.3:
                    landed = True
            elif t == 'HEARTBEAT':
                is_armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not is_armed and landed:
                    print('[MAV] RTL complete — landed and disarmed', flush=True)
                    return True
            elif t == 'STATUSTEXT':
                text = (msg.text.decode('utf-8', errors='replace')
                        if isinstance(msg.text, bytes) else str(msg.text))
                print(f'[MAV] AP: {text.strip()}', flush=True)
        print('[MAV] RTL timeout — proceeding', flush=True)
        return False

    # ── state polling ─────────────────────────────────────────────────────

    def get_state(self):
        """Poll GLOBAL_POSITION_INT, VFR_HUD, and HEARTBEAT non-blockingly.

        Returns a dict with keys: lat, lon, alt_m, heading_deg, airspeed,
        groundspeed, armed.  Missing values are None.
        """
        state = {
            'lat': None, 'lon': None, 'alt_m': None,
            'heading_deg': None, 'airspeed': None,
            'groundspeed': None, 'armed': None,
        }
        deadline = time.time() + 0.2
        while time.time() < deadline:
            msg = self.conn.recv_match(
                type=['GLOBAL_POSITION_INT', 'VFR_HUD', 'HEARTBEAT'],
                blocking=False)
            if msg is None:
                time.sleep(0.02)
                continue
            t = msg.get_type()
            if t == 'GLOBAL_POSITION_INT':
                state['lat'] = msg.lat / 1e7
                state['lon'] = msg.lon / 1e7
                state['alt_m'] = msg.relative_alt / 1000.0
                state['heading_deg'] = msg.hdg / 100.0
            elif t == 'VFR_HUD':
                state['airspeed'] = msg.airspeed
                state['groundspeed'] = msg.groundspeed
            elif t == 'HEARTBEAT':
                state['armed'] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return state

    def format_state_text(self, state):
        """Return a compact English string describing the drone state for the LLM."""
        parts = []
        if state.get('alt_m') is not None:
            parts.append(f"altitude {state['alt_m']:.1f}m")
        if state.get('heading_deg') is not None:
            parts.append(f"heading {state['heading_deg']:.0f}°")
        if state.get('groundspeed') is not None:
            parts.append(f"speed {state['groundspeed']:.1f}m/s")
        if state.get('armed') is not None:
            parts.append('armed' if state['armed'] else 'disarmed')
        return 'Drone state: ' + (', '.join(parts) if parts else 'unknown')
