#!/usr/bin/env python3
"""
fly_tour.py — fly the vtol_description drone on a scenic tour of the
neighborhood world, driving the Gazebo model pose directly (no MAVLink).

Reuses the proven approach from mission.py (set_pose service + prop cmd_vel
topics + a 60 Hz anti-flicker pose-stream thread), but flies a full circuit
over the ring roads and houses and yaws the aircraft to face its direction
of travel.

Sequence:  spool up -> vertical takeoff -> transit out -> loop around the
neighborhood -> return to center -> vertical land -> spool down.

Run while the sim is up (default gz partition):
    source /opt/ros/humble/setup.bash
    source ~/ardu_ws/install/setup.bash
    python3 fly_tour.py
"""
import math
import sys
import threading
import time

import gz.transport13     as transport
import gz.msgs10.pose_pb2     as pose_pb2
import gz.msgs10.boolean_pb2  as boolean_pb2
import gz.msgs10.double_pb2   as double_pb2

WORLD   = 'vtol_world'
MODEL   = 'vtol'
SERVICE = f'/world/{WORLD}/set_pose'
GROUND_Z = 0.333
_h = math.sqrt(2) / 2                      # belly-down base: Roll +90 deg about X

STREAM_HZ = 60
PROP_JOINTS = [('prop_1_joint', +50.0), ('prop_2_joint', -50.0),
               ('prop_3_joint', +50.0), ('prop_4_joint', -50.0),
               ('pusher_joint',  +30.0)]

_node = transport.Node()
_prop_pubs = {}

# shared target (path thread writes, stream thread reads)
_tx, _ty, _tz, _tyaw = 0.0, 0.0, GROUND_Z, 0.0
_lock = threading.Lock()
_alive = True


def _init_props():
    for name, _ in PROP_JOINTS:
        _prop_pubs[name] = _node.advertise(
            f'/model/{MODEL}/joint/{name}/cmd_vel', double_pb2.Double)


def _set_props(scale):
    msg = double_pb2.Double()
    for name, mx in PROP_JOINTS:
        msg.data = mx * float(scale)
        _prop_pubs[name].publish(msg)


def _yaw_quat(psi):
    """Belly-down orientation composed with a world-Z yaw of psi.
    q = Rz(psi) (x) Rx(90deg)  ->  (c*h, c*h, s*h, s*h), c=cos(psi/2) s=sin(psi/2)."""
    c, s = math.cos(psi / 2.0), math.sin(psi / 2.0)
    return (c * _h, c * _h, s * _h, s * _h)


def _set_pose(x, y, z, psi, timeout_ms=80):
    p = pose_pb2.Pose()
    p.name = MODEL
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    w, qx, qy, qz = _yaw_quat(psi)
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = w, qx, qy, qz
    ok, _ = _node.request(SERVICE, p, pose_pb2.Pose, boolean_pb2.Boolean, timeout_ms)
    return ok


def _stream_thread():
    dt = 1.0 / STREAM_HZ
    while _alive:
        t0 = time.monotonic()
        with _lock:
            x, y, z, psi = _tx, _ty, _tz, _tyaw
        _set_pose(x, y, z, psi, timeout_ms=80)
        gap = dt - (time.monotonic() - t0)
        if gap > 0:
            time.sleep(gap)


# ── path construction ────────────────────────────────────────────────────────
ALT = 24.0                 # cruise altitude AGL (m) — clears the houses
Z_HI = GROUND_Z + ALT
R_H = 50.0                 # house-ring radius the figure-eight lobes trace
LOOPS = 2                  # number of figure-eights to fly
H_SPEED = 10.0             # horizontal speed (m/s)

# House positions in the world (must match the world SDF include poses).
HOUSES = [('N', 0.0, 50.0), ('E', 50.0, 2.0), ('S', 2.0, -50.0),
          ('W', -50.0, 0.0), ('NW', -37.0, 37.0)]


def _seg_line(p0, p1, speed):
    (x0, y0, z0), (x1, y1, z1) = p0, p1
    dist = math.dist((x0, y0, z0), (x1, y1, z1))
    n = max(2, int(dist / speed * STREAM_HZ))
    return [(x0 + (x1 - x0) * i / n,
             y0 + (y1 - y0) * i / n,
             z0 + (z1 - z0) * i / n) for i in range(n + 1)]


def _seg_arc(cx, cy, z, r, a0, a1, speed):
    arc = abs(a1 - a0) * r
    n = max(2, int(arc / speed * STREAM_HZ))
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n),
             cy + r * math.sin(a0 + (a1 - a0) * i / n), z)
            for i in range(n + 1)]


def _figure_eight():
    """One figure-eight over the houses, starting/ending at the center (0,0).

    Crossover at the origin; the NW lobe arcs over the N, NW and W houses and
    the SE lobe arcs over the E and S houses, joined by the two crossing
    diameters (vertical S<->N and horizontal W<->E)."""
    hp, r, s = math.pi / 2, R_H, H_SPEED
    seg = []
    seg += _seg_line((0, 0, Z_HI), (0, r, Z_HI), speed=s)        # center -> N
    seg += _seg_arc(0, 0, Z_HI, r, hp, math.pi, speed=s)         # N -> NW -> W (CCW)
    seg += _seg_line((-r, 0, Z_HI), (r, 0, Z_HI), speed=s)       # W -> center -> E
    seg += _seg_arc(0, 0, Z_HI, r, 0.0, -hp, speed=s)            # E -> SE -> S (CW)
    seg += _seg_line((0, -r, Z_HI), (0, 0, Z_HI), speed=s)       # S -> center
    return seg


def _smooth_yaw(pts, alpha=0.10):
    """Heading from horizontal velocity (psi = theta + 90deg), low-pass filtered
    so the ~90 deg turns at the lobe corners are flown as smooth banks."""
    out, psi = [], math.pi / 2
    for i, (x, y, z) in enumerate(pts):
        if i + 1 < len(pts):
            dx, dy = pts[i + 1][0] - x, pts[i + 1][1] - y
            if dx * dx + dy * dy > 1e-6:
                target = math.atan2(dy, dx) + math.pi / 2
                d = (target - psi + math.pi) % (2 * math.pi) - math.pi   # wrap
                psi += alpha * d
        out.append((x, y, z, psi))
    return out


def build_path():
    pts = []
    pts += _seg_line((0, 0, GROUND_Z), (0, 0, Z_HI), speed=3.0)      # takeoff
    for _ in range(LOOPS):
        pts += _figure_eight()                                      # figure-eights
    pts += _seg_line((0, 0, Z_HI), (0, 0, GROUND_Z), speed=3.0)      # land
    path = _smooth_yaw(pts)
    _report_house_coverage(path)
    return path


def _report_house_coverage(path):
    print('[tour] figure-eight closest approach to each house '
          f'(alt {ALT:.0f} m):', flush=True)
    for name, hx, hy in HOUSES:
        d = min(math.hypot(x - hx, y - hy) for x, y, _, _ in path)
        print(f'         {name:>2} ({hx:+.0f},{hy:+.0f}): {d:4.1f} m', flush=True)


def main():
    global _tx, _ty, _tz, _tyaw, _alive
    print('[tour] connecting to Gazebo ...', flush=True)
    _init_props()
    time.sleep(0.4)
    for k in range(40):
        if _set_pose(0, 0, GROUND_Z, math.pi / 2):
            break
        if k % 4 == 0:
            print(f'  waiting for {SERVICE} (is the sim running?)', flush=True)
        time.sleep(0.5)
    else:
        sys.exit('[tour] ERROR: set_pose service unreachable')
    print('[tour] connected.', flush=True)

    threading.Thread(target=_stream_thread, daemon=True).start()

    path = build_path()
    print(f'[tour] spooling up props', flush=True)
    for i in range(31):
        _set_props(i / 30.0)
        time.sleep(1.2 / 30)
    _set_props(1.0)

    print(f'[tour] flying {len(path)} waypoints '
          f'(~{len(path)/STREAM_HZ:.0f}s): takeoff -> loop -> land', flush=True)
    dt = 1.0 / STREAM_HZ
    t0 = time.monotonic()
    for i, (x, y, z, psi) in enumerate(path):
        with _lock:
            _tx, _ty, _tz, _tyaw = x, y, z, psi
        nxt = t0 + (i + 1) * dt
        gap = nxt - time.monotonic()
        if gap > 0:
            time.sleep(gap)

    with _lock:
        _tx, _ty, _tz, _tyaw = 0.0, 0.0, GROUND_Z, math.pi / 2
    print('[tour] landed. spooling down props', flush=True)
    for i in range(31):
        _set_props(1.0 - i / 30.0)
        time.sleep(1.2 / 30)
    _set_props(0.0)
    time.sleep(0.5)
    _alive = False
    print('[tour] done.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        _alive = False
        _set_props(0.0)
        _set_pose(0, 0, GROUND_Z, math.pi / 2, timeout_ms=1000)
        print('\n[tour] aborted — drone returned to ground.', flush=True)
