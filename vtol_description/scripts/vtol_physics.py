#!/usr/bin/env python3
"""
vtol_physics.py — 6-DOF rigid body physics for the VTOL radio controller.

Simulates gravity, rotor thrust, aerodynamic drag, wind, and ground contact.
The Gazebo model remains kinematic — we compute state in Python and apply via set_pose.

Usage:
    # Smoke tests:
    python3 vtol_physics.py
"""

import math
import numpy as np

# ── Quaternion utilities (w, x, y, z convention) ─────────────────────────────

def quat_multiply(q1, q2):
    """Hamilton product of two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_conjugate(q):
    """Conjugate (inverse for unit quaternion)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    rotated = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return rotated[1:4]


def quat_from_euler(roll, pitch, yaw):
    """Create quaternion from Euler angles (ZYX convention)."""
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def quat_to_euler(q):
    """Extract roll, pitch, yaw from quaternion (ZYX)."""
    w, x, y, z = q
    # Roll (X)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (Y)
    sinp = 2.0 * (w*y - z*x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # Yaw (Z)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_normalize(q):
    """Normalize quaternion to unit length."""
    n = np.linalg.norm(q)
    if n < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


# ── Flight mode enum ─────────────────────────────────────────────────────────

MODE_QUAD = 'QUAD'
MODE_TRANSITION = 'TRANSITION'
MODE_PLANE = 'PLANE'


# ── Default physics parameters ───────────────────────────────────────────────

DEFAULT_CONFIG = {
    'mass': 8.1,            # kg
    'twr': 2.0,             # thrust-to-weight ratio at full throttle
    'I_roll': 0.5,          # kg*m^2
    'I_pitch': 0.3,         # kg*m^2
    'I_yaw': 0.1,           # kg*m^2
    'roll_gain': 0.4,       # differential thrust authority for roll
    'pitch_gain': 0.4,      # differential thrust authority for pitch
    'yaw_gain': 0.2,        # reaction torque authority for yaw
    'drag_cd_a': 0.06,      # drag coefficient * frontal area (m^2)
    'ang_drag': 0.3,        # angular drag coefficient
    'motor_tau': 0.05,      # motor response time constant (s)
    'max_tilt_deg': 45.0,   # max tilt safety clamp (QUAD mode)
    'ground_z': 0.333,      # ground level in Gazebo
    'ground_friction': 5.0, # lateral friction on ground
    'auto_level_rate': 3.0, # rad/s auto-level on ground
    # Wing aerodynamics
    'wing_area': 0.45,          # m^2
    'wing_cl_alpha': 5.0,       # lift curve slope (per radian)
    'wing_cl_max': 1.4,         # max CL before stall
    'wing_cd0': 0.03,           # parasitic drag coefficient
    'wing_oswald': 0.8,         # Oswald efficiency factor
    'wing_ar': 8.0,             # aspect ratio
    # Transition thresholds
    'transition_speed': 12.0,       # m/s airspeed to complete fwd transition
    'back_transition_speed': 8.0,   # m/s airspeed to complete back transition
    'stall_speed': 10.0,            # m/s — auto back-transition if below this in PLANE
    'transition_rate': 0.3,         # alpha change per second during transition
    # Control surface authority (scaled by dynamic pressure)
    'aileron_authority': 3.0,   # rad/s^2 per unit input at full q
    'elevator_authority': 2.5,  # rad/s^2 per unit input at full q
    'rudder_authority': 1.5,    # rad/s^2 per unit input at full q
    # Plane mode tilt limits
    'plane_max_roll_deg': 60.0,
    'plane_max_pitch_deg': 30.0,
}


# ── VTOLPhysics class ────────────────────────────────────────────────────────

class VTOLPhysics:
    """6-DOF rigid body physics for VTOL quadrotor with quad/plane transition."""

    def __init__(self, config=None):
        cfg = dict(DEFAULT_CONFIG)
        if config:
            cfg.update(config)

        self.mass = cfg['mass']
        self.gravity = 9.81
        self.max_thrust = self.mass * self.gravity * cfg['twr']  # total max thrust (N)
        self.I = np.array([cfg['I_roll'], cfg['I_pitch'], cfg['I_yaw']])
        self.roll_gain = cfg['roll_gain']
        self.pitch_gain = cfg['pitch_gain']
        self.yaw_gain = cfg['yaw_gain']
        self.drag_cd_a = cfg['drag_cd_a']
        self.ang_drag = cfg['ang_drag']
        self.motor_tau = cfg['motor_tau']
        self.max_tilt = math.radians(cfg['max_tilt_deg'])
        self.ground_z = cfg['ground_z']
        self.ground_friction = cfg['ground_friction']
        self.auto_level_rate = cfg['auto_level_rate']

        # Wing aerodynamics
        self.wing_area = cfg['wing_area']
        self.wing_cl_alpha = cfg['wing_cl_alpha']
        self.wing_cl_max = cfg['wing_cl_max']
        self.wing_cd0 = cfg['wing_cd0']
        self.wing_oswald = cfg['wing_oswald']
        self.wing_ar = cfg['wing_ar']

        # Transition parameters
        self.transition_speed = cfg['transition_speed']
        self.back_transition_speed = cfg['back_transition_speed']
        self.stall_speed = cfg['stall_speed']
        self.transition_rate = cfg['transition_rate']

        # Control surface authority
        self.aileron_authority = cfg['aileron_authority']
        self.elevator_authority = cfg['elevator_authority']
        self.rudder_authority = cfg['rudder_authority']

        # Plane mode limits
        self.plane_max_roll = math.radians(cfg['plane_max_roll_deg'])
        self.plane_max_pitch = math.radians(cfg['plane_max_pitch_deg'])

        # State
        self.position = np.array([0.0, 0.0, self.ground_z])
        self.velocity = np.zeros(3)  # world frame m/s
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion (w,x,y,z) — level
        self.ang_velocity = np.zeros(3)  # body frame rad/s (roll, pitch, yaw)

        # Flight mode state
        self.mode = MODE_QUAD
        self.transition_alpha = 0.0  # 0 = full quad, 1 = full plane
        self.transition_direction = 0  # +1 = quad→plane, -1 = plane→quad
        self.stall_timer = 0.0  # time below stall speed in PLANE mode

        # Motor state: 4 lift rotors [FL, FR, BL, BR]
        self.rotor_speeds = np.zeros(4)  # 0..1 normalized
        self._rotor_targets = np.zeros(4)

        # Pusher motor
        self.pusher_speed = 0.0
        self._pusher_target = 0.0
        self.pusher_thrust_max = self.mass * self.gravity * 0.5  # max forward thrust

        # Control surface deflections (for animation feedback)
        self.aileron_deflection = 0.0   # [-1, 1]
        self.elevator_deflection = 0.0  # [-1, 1]
        self.rudder_deflection = 0.0    # [-1, 1]

        # Wind
        self.wind_velocity = np.zeros(3)  # constant wind (world frame, m/s)
        self.gust_magnitude = 0.0
        self._gust_current = np.zeros(3)
        self._gust_timer = 0.0

        # Inputs (set each frame)
        self._throttle = 0.0
        self._roll_input = 0.0
        self._pitch_input = 0.0
        self._yaw_input = 0.0
        self._pusher_input = 0.0

    def set_inputs(self, throttle, roll, pitch, yaw, pusher=0.0):
        """Set control inputs. All values in [-1, 1] except throttle [0, 1]."""
        self._throttle = max(0.0, min(1.0, throttle))
        self._roll_input = max(-1.0, min(1.0, roll))
        self._pitch_input = max(-1.0, min(1.0, pitch))
        self._yaw_input = max(-1.0, min(1.0, yaw))
        self._pusher_input = max(0.0, min(1.0, pusher))

    def start_transition(self):
        """Initiate a flight mode transition (QUAD→PLANE or PLANE→QUAD)."""
        if self.mode == MODE_QUAD:
            self.mode = MODE_TRANSITION
            self.transition_direction = +1  # quad → plane
        elif self.mode == MODE_PLANE:
            self.mode = MODE_TRANSITION
            self.transition_direction = -1  # plane → quad
        # If already in TRANSITION, ignore (let it complete)

    def _update_transition(self, dt):
        """Advance transition alpha and check airspeed thresholds."""
        if self.mode != MODE_TRANSITION:
            # Stall protection in PLANE mode
            if self.mode == MODE_PLANE:
                airspeed = self.get_airspeed()
                if airspeed < self.stall_speed:
                    self.stall_timer += dt
                    if self.stall_timer > 1.0:
                        # Auto back-transition
                        self.mode = MODE_TRANSITION
                        self.transition_direction = -1
                        self.stall_timer = 0.0
                else:
                    self.stall_timer = 0.0
            return

        airspeed = self.get_airspeed()

        if self.transition_direction > 0:
            # QUAD → PLANE: ramp alpha up
            self.transition_alpha += self.transition_rate * dt
            self.transition_alpha = min(self.transition_alpha, 1.0)
            # Complete when airspeed exceeds threshold
            if airspeed >= self.transition_speed and self.transition_alpha >= 0.8:
                self.transition_alpha = 1.0
                self.mode = MODE_PLANE
                self.transition_direction = 0
        else:
            # PLANE → QUAD: ramp alpha down
            self.transition_alpha -= self.transition_rate * dt
            self.transition_alpha = max(self.transition_alpha, 0.0)
            # Complete when airspeed drops below threshold
            if airspeed <= self.back_transition_speed and self.transition_alpha <= 0.2:
                self.transition_alpha = 0.0
                self.mode = MODE_QUAD
                self.transition_direction = 0

    def _compute_wing_forces(self):
        """Compute wing lift and drag based on airspeed and angle of attack."""
        rho = 1.225
        airspeed_vec = self.velocity - self.wind_velocity - self._gust_current
        speed = np.linalg.norm(airspeed_vec)

        force_world = np.zeros(3)
        if speed < 0.5:
            return force_world

        # Velocity in body frame
        vel_body = quat_rotate(quat_conjugate(self.orientation), airspeed_vec)

        # Angle of attack: angle between velocity and body X axis
        # In our convention: X=forward, Z=up in body frame
        # Positive AoA when wind comes from below (vel_body_z > 0 = wind has upward component in body)
        aoa = math.atan2(vel_body[2], vel_body[0]) if abs(vel_body[0]) > 0.1 else 0.0

        # Lift coefficient
        cl = self.wing_cl_alpha * aoa
        cl = max(-self.wing_cl_max, min(self.wing_cl_max, cl))

        # Induced drag
        cd_i = cl * cl / (math.pi * self.wing_ar * self.wing_oswald)
        cd = self.wing_cd0 + cd_i

        # Dynamic pressure
        q = 0.5 * rho * speed * speed

        # Lift: perpendicular to velocity in the vertical plane (body Z direction rotated to world)
        lift_mag = q * self.wing_area * cl
        # Lift direction: perpendicular to velocity, in the plane containing velocity and body-Z
        body_up = quat_rotate(self.orientation, np.array([0.0, 0.0, 1.0]))
        vel_dir = airspeed_vec / speed
        # Lift perpendicular to velocity, biased toward body up
        lift_dir = body_up - np.dot(body_up, vel_dir) * vel_dir
        lift_norm = np.linalg.norm(lift_dir)
        if lift_norm > 0.01:
            lift_dir /= lift_norm
            force_world += lift_dir * lift_mag

        # Drag: opposes velocity
        drag_mag = q * self.wing_area * cd
        force_world -= vel_dir * drag_mag

        return force_world

    def _compute_surface_torques(self):
        """Compute control surface torques (scaled by dynamic pressure)."""
        rho = 1.225
        airspeed_vec = self.velocity - self.wind_velocity - self._gust_current
        speed = np.linalg.norm(airspeed_vec)

        torque_body = np.zeros(3)
        if speed < 1.0:
            return torque_body

        # Dynamic pressure normalized (effectiveness scales with q)
        # Reference speed for full authority = transition_speed
        q_factor = min(1.0, (speed / self.transition_speed) ** 2)

        # Aileron → roll torque
        torque_body[0] = self._roll_input * self.aileron_authority * q_factor * self.I[0]
        # Elevator → pitch torque
        torque_body[1] = self._pitch_input * self.elevator_authority * q_factor * self.I[1]
        # Rudder → yaw torque
        torque_body[2] = self._yaw_input * self.rudder_authority * q_factor * self.I[2]

        # Store deflections for animation
        self.aileron_deflection = self._roll_input
        self.elevator_deflection = self._pitch_input
        self.rudder_deflection = self._yaw_input

        return torque_body

    def _mix_motors(self):
        """Compute individual rotor targets from collective + differential inputs."""
        if self.mode == MODE_PLANE:
            # Lift rotors off, pusher = throttle
            self._rotor_targets[:] = 0.0
            self._pusher_target = self._throttle
            return
        elif self.mode == MODE_TRANSITION:
            if self.transition_direction > 0:
                # Quad→Plane: rotors hold altitude (hover throttle), pusher full
                base = 0.5  # hover throttle regardless of input
                self._pusher_target = 1.0
            else:
                # Plane→Quad: rotors spool up, pusher reduces
                base = self._throttle
                self._pusher_target = max(0.0, 1.0 - (1.0 - self.transition_alpha))
            # Scale rotor authority by (1 - alpha)
            rotor_scale = 1.0 - self.transition_alpha
            roll_diff = self._roll_input * self.roll_gain * 0.5 * rotor_scale
            pitch_diff = self._pitch_input * self.pitch_gain * 0.5 * rotor_scale
            yaw_diff = self._yaw_input * self.yaw_gain * 0.5 * rotor_scale
            self._rotor_targets[0] = base * rotor_scale + roll_diff + pitch_diff - yaw_diff
            self._rotor_targets[1] = base * rotor_scale - roll_diff + pitch_diff + yaw_diff
            self._rotor_targets[2] = base * rotor_scale + roll_diff - pitch_diff + yaw_diff
            self._rotor_targets[3] = base * rotor_scale - roll_diff - pitch_diff - yaw_diff
            self._rotor_targets = np.clip(self._rotor_targets, 0.0, 1.0)
            return

        # QUAD mode (unchanged)
        base = self._throttle
        # Mixer: FL, FR, BL, BR
        # Roll: +left, -right
        # Pitch: +front, -back
        # Yaw: differential CW/CCW torque
        roll_diff = self._roll_input * self.roll_gain * 0.5
        pitch_diff = self._pitch_input * self.pitch_gain * 0.5
        yaw_diff = self._yaw_input * self.yaw_gain * 0.5

        self._rotor_targets[0] = base + roll_diff + pitch_diff - yaw_diff  # FL (CCW)
        self._rotor_targets[1] = base - roll_diff + pitch_diff + yaw_diff  # FR (CW)
        self._rotor_targets[2] = base + roll_diff - pitch_diff + yaw_diff  # BL (CW)
        self._rotor_targets[3] = base - roll_diff - pitch_diff - yaw_diff  # BR (CCW)

        # Clamp
        self._rotor_targets = np.clip(self._rotor_targets, 0.0, 1.0)

        # Pusher
        self._pusher_target = self._pusher_input

    def _update_motors(self, dt):
        """First-order lag on motor speeds."""
        alpha = 1.0 - math.exp(-dt / self.motor_tau)
        self.rotor_speeds += (self._rotor_targets - self.rotor_speeds) * alpha
        self.pusher_speed += (self._pusher_target - self.pusher_speed) * alpha

    def _compute_forces_torques(self):
        """Compute net force (world) and torque (body)."""
        # ── Gravity (world frame) ──
        force_world = np.array([0.0, 0.0, -self.mass * self.gravity])

        # ── Rotor thrust (body Z-up, then rotated to world) ──
        # Scaled by (1 - transition_alpha) so rotors fade during transition
        rotor_scale = 1.0 - self.transition_alpha
        total_thrust = np.sum(self.rotor_speeds) * (self.max_thrust / 4.0) * rotor_scale
        thrust_body = np.array([0.0, 0.0, total_thrust])
        thrust_world = quat_rotate(self.orientation, thrust_body)
        force_world += thrust_world

        # ── Pusher thrust (body forward = body X in our convention) ──
        if self.pusher_speed > 0.01:
            pusher_force_body = np.array([self.pusher_speed * self.pusher_thrust_max, 0.0, 0.0])
            force_world += quat_rotate(self.orientation, pusher_force_body)

        # ── Wing lift and drag ──
        wing_forces = self._compute_wing_forces()
        force_world += wing_forces

        # ── Fuselage aerodynamic drag (world frame, quadratic) ──
        airspeed = self.velocity - self.wind_velocity - self._gust_current
        speed = np.linalg.norm(airspeed)
        if speed > 0.01:
            rho = 1.225  # air density kg/m^3
            drag_mag = 0.5 * rho * self.drag_cd_a * speed * speed
            force_world -= (airspeed / speed) * drag_mag

        # ── Torques (body frame) ──
        torque_body = np.zeros(3)

        # Rotor torques (scaled by rotor_scale)
        if rotor_scale > 0.01:
            # Roll torque from differential left/right
            roll_moment = (self.rotor_speeds[0] + self.rotor_speeds[2]
                           - self.rotor_speeds[1] - self.rotor_speeds[3])
            torque_body[0] += roll_moment * (self.max_thrust / 4.0) * 0.3 * rotor_scale

            # Pitch torque from differential front/back
            pitch_moment = (self.rotor_speeds[0] + self.rotor_speeds[1]
                            - self.rotor_speeds[2] - self.rotor_speeds[3])
            torque_body[1] += pitch_moment * (self.max_thrust / 4.0) * 0.3 * rotor_scale

            # Yaw torque from reaction torques (CW/CCW)
            yaw_moment = (-self.rotor_speeds[0] + self.rotor_speeds[1]
                          + self.rotor_speeds[2] - self.rotor_speeds[3])
            torque_body[2] += yaw_moment * (self.max_thrust / 4.0) * 0.05 * rotor_scale

        # Control surface torques (significant at speed)
        surface_torques = self._compute_surface_torques()
        # Blend: in QUAD mode surfaces have no authority (q_factor handles this),
        # but we always add them — they're naturally zero at low speed
        torque_body += surface_torques

        # Angular drag
        torque_body -= self.ang_drag * self.ang_velocity

        return force_world, torque_body

    def _update_wind(self, dt):
        """Update gust model."""
        if self.gust_magnitude <= 0:
            return
        self._gust_timer -= dt
        if self._gust_timer <= 0:
            # New gust
            direction = np.random.randn(3)
            norm = np.linalg.norm(direction)
            if norm > 0.01:
                direction /= norm
            self._gust_current = direction * self.gust_magnitude * np.random.uniform(0.5, 1.0)
            self._gust_timer = np.random.uniform(1.0, 4.0)
        else:
            # Exponential decay
            decay = math.exp(-dt * 2.0)
            self._gust_current *= decay

    def step(self, dt):
        """Advance physics by dt seconds."""
        self._update_transition(dt)
        self._mix_motors()
        self._update_motors(dt)
        self._update_wind(dt)

        force_world, torque_body = self._compute_forces_torques()

        # ── Linear integration (semi-implicit Euler) ──
        accel = force_world / self.mass
        self.velocity += accel * dt
        self.position += self.velocity * dt

        # ── Angular integration ──
        ang_accel = torque_body / self.I
        self.ang_velocity += ang_accel * dt
        self.ang_velocity = np.clip(self.ang_velocity, -5.0, 5.0)  # safety clamp

        # Apply angular velocity to orientation quaternion
        # dq = 0.5 * q * omega_quat
        omega_q = np.array([0.0, self.ang_velocity[0], self.ang_velocity[1], self.ang_velocity[2]])
        dq = 0.5 * quat_multiply(self.orientation, omega_q)
        self.orientation += dq * dt
        self.orientation = quat_normalize(self.orientation)

        # ── Tilt safety clamp (mode-dependent) ──
        roll, pitch, yaw = quat_to_euler(self.orientation)
        clamped = False
        if self.mode == MODE_PLANE or (self.mode == MODE_TRANSITION and self.transition_alpha > 0.5):
            max_roll = self.plane_max_roll
            max_pitch = self.plane_max_pitch
        else:
            max_roll = self.max_tilt
            max_pitch = self.max_tilt
        if abs(roll) > max_roll:
            roll = math.copysign(max_roll, roll)
            clamped = True
        if abs(pitch) > max_pitch:
            pitch = math.copysign(max_pitch, pitch)
            clamped = True
        if clamped:
            self.orientation = quat_from_euler(roll, pitch, yaw)

        # ── Ground contact ──
        if self.position[2] <= self.ground_z:
            self.position[2] = self.ground_z
            # Kill downward velocity
            if self.velocity[2] < 0:
                self.velocity[2] = 0.0
            # Ground friction
            self.velocity[0] *= max(0.0, 1.0 - self.ground_friction * dt)
            self.velocity[1] *= max(0.0, 1.0 - self.ground_friction * dt)
            # Auto-level on ground
            roll_err = -roll
            pitch_err = -pitch
            rate = self.auto_level_rate * dt
            new_roll = roll + np.clip(roll_err, -rate, rate)
            new_pitch = pitch + np.clip(pitch_err, -rate, rate)
            self.orientation = quat_from_euler(new_roll, new_pitch, yaw)
            self.ang_velocity *= 0.5  # damp angular velocity on ground

    def get_euler(self):
        """Get current roll, pitch, yaw in degrees."""
        r, p, y = quat_to_euler(self.orientation)
        return math.degrees(r), math.degrees(p), math.degrees(y)

    def get_airspeed(self):
        """Get airspeed magnitude (m/s)."""
        airspeed = self.velocity - self.wind_velocity - self._gust_current
        return np.linalg.norm(airspeed)

    def get_gazebo_quaternion(self):
        """
        Get quaternion for Gazebo set_pose.

        The VTOL model needs a base rotation of Roll=+90° (pi/2) to stand upright.
        We compose: base_rotation * physics_orientation
        """
        # Base rotation: 90° around X axis → (w=cos(45°), x=sin(45°), y=0, z=0)
        h = math.sqrt(2) / 2
        q_base = np.array([h, h, 0.0, 0.0])
        # Compose: first base rotation, then physics orientation on top
        q_final = quat_multiply(q_base, self.orientation)
        return q_final

    def reset(self, position=None):
        """Reset to initial state."""
        self.position = np.array(position) if position else np.array([0.0, 0.0, self.ground_z])
        self.velocity = np.zeros(3)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.ang_velocity = np.zeros(3)
        self.rotor_speeds = np.zeros(4)
        self._rotor_targets = np.zeros(4)
        self.pusher_speed = 0.0
        self._pusher_target = 0.0
        self.mode = MODE_QUAD
        self.transition_alpha = 0.0
        self.transition_direction = 0
        self.stall_timer = 0.0
        self.aileron_deflection = 0.0
        self.elevator_deflection = 0.0
        self.rudder_deflection = 0.0

    def is_on_ground(self):
        """Check if vehicle is on the ground."""
        return self.position[2] <= self.ground_z + 0.01


# ── Smoke tests ──────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('=== VTOL Physics Smoke Tests ===\n')

    # Test 1: Gravity — vehicle should fall
    print('Test 1: Gravity (no thrust, should fall)')
    phys = VTOLPhysics()
    phys.position[2] = 5.0  # start at 5m
    for _ in range(50):  # 1 second at 50Hz
        phys.step(0.02)
    print(f'  After 1s free-fall: z = {phys.position[2]:.3f} m  (expected ~0.1m fallen)')
    assert phys.position[2] < 5.0, 'Should have fallen!'
    print('  PASS\n')

    # Test 2: Hover equilibrium — 50% throttle should hold altitude
    print('Test 2: Hover at 50% throttle (motors pre-spooled)')
    phys = VTOLPhysics()
    phys.position[2] = 5.0
    phys.velocity[2] = 0.0
    phys.rotor_speeds[:] = 0.5  # pre-spool motors to hover
    for _ in range(250):  # 5 seconds
        phys.set_inputs(throttle=0.5, roll=0.0, pitch=0.0, yaw=0.0)
        phys.step(0.02)
    drift = abs(phys.position[2] - 5.0)
    print(f'  After 5s at 50% throttle: z = {phys.position[2]:.3f} m  (drift = {drift:.4f} m)')
    assert drift < 0.5, f'Hover drift too large: {drift}'
    print('  PASS\n')

    # Test 3: Roll input tilts vehicle
    print('Test 3: Roll input')
    phys = VTOLPhysics()
    phys.position[2] = 5.0
    for _ in range(25):  # 0.5s
        phys.set_inputs(throttle=0.5, roll=1.0, pitch=0.0, yaw=0.0)
        phys.step(0.02)
    roll_deg, _, _ = phys.get_euler()
    print(f'  After 0.5s full roll input: roll = {roll_deg:.1f}°')
    assert abs(roll_deg) > 5.0, 'Should have tilted!'
    print('  PASS\n')

    # Test 4: Ground contact prevents falling through
    print('Test 4: Ground contact')
    phys = VTOLPhysics()
    phys.position[2] = 1.0
    for _ in range(500):  # 10 seconds of falling
        phys.set_inputs(throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0)
        phys.step(0.02)
    print(f'  After 10s with no thrust: z = {phys.position[2]:.3f} m  (ground = {phys.ground_z})')
    assert phys.position[2] >= phys.ground_z - 0.001, 'Fell through ground!'
    print('  PASS\n')

    # Test 5: Yaw rotates vehicle
    print('Test 5: Yaw input')
    phys = VTOLPhysics()
    phys.position[2] = 5.0
    for _ in range(50):  # 1s
        phys.set_inputs(throttle=0.5, roll=0.0, pitch=0.0, yaw=1.0)
        phys.step(0.02)
    _, _, yaw_deg = phys.get_euler()
    print(f'  After 1s full yaw input: yaw = {yaw_deg:.1f}°')
    assert abs(yaw_deg) > 2.0, 'Should have rotated!'
    print('  PASS\n')

    # Test 6: Transition QUAD → PLANE
    print('Test 6: Forward transition (QUAD → PLANE)')
    phys = VTOLPhysics({'transition_rate': 0.5})  # faster for test
    phys.position[2] = 20.0
    phys.rotor_speeds[:] = 0.5
    # Give it forward velocity to simulate acceleration
    phys.velocity[0] = 5.0
    phys.start_transition()
    assert phys.mode == MODE_TRANSITION, f'Expected TRANSITION, got {phys.mode}'
    assert phys.transition_direction == +1, 'Should be forward transition'
    # Run until transition completes or timeout
    for i in range(500):  # 10s max
        phys.set_inputs(throttle=0.5, roll=0.0, pitch=0.0, yaw=0.0)
        phys.step(0.02)
        # Simulate increasing airspeed (pusher thrust will do this naturally)
        if phys.get_airspeed() < 15.0:
            phys.velocity[0] += 0.1  # help accelerate for test
        if phys.mode == MODE_PLANE:
            break
    print(f'  Transition completed in {(i+1)*0.02:.1f}s, airspeed={phys.get_airspeed():.1f} m/s')
    assert phys.mode == MODE_PLANE, f'Did not reach PLANE mode (mode={phys.mode}, alpha={phys.transition_alpha:.2f})'
    assert phys.transition_alpha == 1.0, f'Alpha should be 1.0, got {phys.transition_alpha}'
    print('  PASS\n')

    # Test 7: Back transition PLANE → QUAD
    print('Test 7: Back transition (PLANE → QUAD)')
    phys.start_transition()
    assert phys.mode == MODE_TRANSITION, f'Expected TRANSITION, got {phys.mode}'
    assert phys.transition_direction == -1, 'Should be back transition'
    # Decelerate
    for i in range(500):
        phys.set_inputs(throttle=0.3, roll=0.0, pitch=0.0, yaw=0.0)
        phys.step(0.02)
        # Simulate deceleration
        phys.velocity[0] *= 0.98
        if phys.mode == MODE_QUAD:
            break
    print(f'  Back-transition completed in {(i+1)*0.02:.1f}s, airspeed={phys.get_airspeed():.1f} m/s')
    assert phys.mode == MODE_QUAD, f'Did not reach QUAD mode (mode={phys.mode}, alpha={phys.transition_alpha:.2f})'
    assert phys.transition_alpha == 0.0, f'Alpha should be 0.0, got {phys.transition_alpha}'
    print('  PASS\n')

    # Test 8: Stall protection in PLANE mode
    print('Test 8: Stall protection (auto back-transition)')
    phys = VTOLPhysics()
    phys.position[2] = 20.0
    # Force into PLANE mode directly for testing
    phys.mode = MODE_PLANE
    phys.transition_alpha = 1.0
    phys.velocity = np.array([5.0, 0.0, 0.0])  # below stall_speed (10 m/s)
    for i in range(100):  # 2s — should trigger after 1s
        phys.set_inputs(throttle=0.0, roll=0.0, pitch=0.0, yaw=0.0)
        phys.step(0.02)
        # Pin velocity to stay below stall (prevent gravity from increasing norm)
        phys.velocity = np.array([5.0, 0.0, 0.0])
        phys.position[2] = 20.0  # prevent ground contact
        if phys.mode == MODE_TRANSITION:
            break
    print(f'  Stall detected after {(i+1)*0.02:.1f}s, mode={phys.mode}')
    assert phys.mode == MODE_TRANSITION, 'Should have auto-triggered back-transition'
    assert phys.transition_direction == -1, 'Should be back transition'
    print('  PASS\n')

    # Test 9: Wing generates lift at speed
    print('Test 9: Wing lift at speed')
    phys = VTOLPhysics()
    phys.position[2] = 20.0
    phys.velocity[0] = 15.0  # flying forward fast
    phys.mode = MODE_PLANE
    phys.transition_alpha = 1.0
    # Slight nose-up for angle of attack
    phys.orientation = quat_from_euler(0.0, 0.05, 0.0)  # ~3° pitch up
    wing_f = phys._compute_wing_forces()
    print(f'  Wing force at 15 m/s, AoA~3°: [{wing_f[0]:.1f}, {wing_f[1]:.1f}, {wing_f[2]:.1f}] N')
    assert wing_f[2] > 0, 'Wing should produce upward lift with positive AoA'
    print('  PASS\n')

    # Test 10: PLANE mode — rotors off, pusher = throttle
    print('Test 10: PLANE mode motor mixing')
    phys = VTOLPhysics()
    phys.mode = MODE_PLANE
    phys.transition_alpha = 1.0
    phys.set_inputs(throttle=0.7, roll=0.5, pitch=0.3, yaw=0.2)
    phys._mix_motors()
    print(f'  Rotor targets: {phys._rotor_targets}')
    print(f'  Pusher target: {phys._pusher_target:.2f}')
    assert np.all(phys._rotor_targets == 0.0), 'Rotors should be off in PLANE mode'
    assert abs(phys._pusher_target - 0.7) < 0.01, f'Pusher should be 0.7, got {phys._pusher_target}'
    print('  PASS\n')

    print('=== All smoke tests passed! ===')
