"""
Fly drone1 around the obstacle course (slalom pylons -> arch1 -> arch2 -> wall -> platform).

Coordinate mapping (Gazebo ENU -> ArduPilot NED):
    NED.x (North) =  Gazebo.y
    NED.y (East)  =  Gazebo.x
    NED.z (Down)  = -Gazebo.z

Obstacle course layout (Gazebo coords):
    pylons      x=40 y=-8/0/+8 and x=45 y=-4/+4     (h=1.5m)
    arch_1      x=52 y=0  (opens along X, fly through facing +X) (h=4m, bar at z=4)
    arch_2      x=60 y=0  (opens along Y, rotated 90, fly through facing +Y or -Y)
    low_wall    x=65 y=0  (h=1.5m, length along Y)
    platform    x=70 y=0  (h=2m, top at z=2)
"""
import argparse, subprocess, time, sys
from pymavlink import mavutil

DEFAULT_URI = "tcp:127.0.0.1:5760"
TAKEOFF_ALT = 5.0      # metres above home
CRUISE_ALT  = 3.0      # below arch bars (4m), above pylons (1.5m)
WALL_ALT    = 4.0      # clear the 1.5m wall
PLATFORM_HOVER = 3.5   # hover above platform (top at 2m)
POS_TOL = 1.0          # m, waypoint reached tolerance


HOME_GX = 0.0   # set from CLI; drone's spawn position in Gazebo X
HOME_GY = 0.0   # set from CLI; drone's spawn position in Gazebo Y


def gz_to_ned(gx, gy, gz):
    """Gazebo ENU absolute -> ArduPilot local NED (relative to drone home)."""
    return (gy - HOME_GY, gx - HOME_GX, -gz)


def reset_pose(world, model, gx, gy, gz=0.3, yaw_rad=1.5708):
    """Snap the Gazebo model back to upright at (gx, gy, gz) with given yaw.
    Quaternion for pure yaw: (0, 0, sin(y/2), cos(y/2)).
    """
    import math
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    req = (f'name: "{model}", '
           f'position: {{x: {gx}, y: {gy}, z: {gz}}}, '
           f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}')
    print(f"[reset] {model} -> ({gx},{gy},{gz}) yaw={math.degrees(yaw_rad):.1f}deg", flush=True)
    r = subprocess.run([
        "gz", "service",
        "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ], capture_output=True, text=True)
    if "data: true" not in r.stdout:
        print(f"[reset] WARN service reply: stdout={r.stdout!r} stderr={r.stderr!r}", flush=True)
    else:
        print("[reset] ok", flush=True)


def wait_heartbeat(m):
    print(f"waiting heartbeat...", flush=True)
    m.wait_heartbeat()
    print(f"connected sys={m.target_system} comp={m.target_component}", flush=True)


def set_mode(m, mode):
    m.set_mode(mode)
    # confirm
    for _ in range(20):
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and mavutil.mode_string_v10(hb) == mode:
            print(f"mode -> {mode}", flush=True)
            return True
    print(f"WARN: mode never confirmed as {mode}", flush=True)
    return False


def arm(m):
    print("arming...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    m.motors_armed_wait()
    print("armed", flush=True)


def takeoff(m, alt):
    print(f"takeoff to {alt}m...", flush=True)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt)
    # wait until reaches >95% of alt
    t0 = time.time()
    while time.time() - t0 < 30:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            rel = msg.relative_alt / 1000.0
            if rel >= 0.95 * alt:
                print(f"reached alt={rel:.2f}m", flush=True)
                return True
    print("WARN: takeoff timeout", flush=True)
    return False


def goto_ned(m, n, e, d, label="", timeout=25):
    print(f"-> {label} NED=({n:+.1f},{e:+.1f},{d:+.1f})", flush=True)
    type_mask = 0b110111111000   # position only
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        n, e, d,
        0, 0, 0,
        0, 0, 0,
        0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
        if msg:
            dx, dy, dz = msg.x - n, msg.y - e, msg.z - d
            err = (dx*dx + dy*dy + dz*dz) ** 0.5
            if err < POS_TOL:
                print(f"   reached (err={err:.2f}m)", flush=True)
                return True
    print(f"   WARN: not reached, continuing", flush=True)
    return False


def goto_gz(m, gx, gy, gz, label="", timeout=25):
    n, e, d = gz_to_ned(gx, gy, gz)
    return goto_ned(m, n, e, d, label, timeout)


def land(m):
    print("LAND mode", flush=True)
    set_mode(m, "LAND")
    t0 = time.time()
    while time.time() - t0 < 30:
        if not m.motors_armed():
            print("disarmed - landed", flush=True)
            return
        time.sleep(0.5)


def main():
    global HOME_GX, HOME_GY
    ap = argparse.ArgumentParser()
    ap.add_argument("--uri", default=DEFAULT_URI, help="MAVLink URI (tcp:127.0.0.1:5760 for drone1, 5770 for drone2)")
    ap.add_argument("--home-x", type=float, default=0.0, help="drone spawn Gazebo X (0 for drone1, 5 for drone2)")
    ap.add_argument("--home-y", type=float, default=0.0, help="drone spawn Gazebo Y")
    ap.add_argument("--alt-offset", type=float, default=0.0, help="add this to every cruise/wall/platform altitude (use to separate drones)")
    ap.add_argument("--world", default="iris_tracking_two", help="Gazebo world name")
    ap.add_argument("--model", default=None, help="Gazebo model name (default: iris for 5760, iris2 otherwise)")
    ap.add_argument("--no-reset", action="store_true", help="skip auto pose-reset before flying")
    args = ap.parse_args()
    HOME_GX, HOME_GY = args.home_x, args.home_y
    if args.model is None:
        args.model = "iris" if "5760" in args.uri else "iris2"
    global CRUISE_ALT, WALL_ALT, PLATFORM_HOVER, TAKEOFF_ALT
    CRUISE_ALT     += args.alt_offset
    WALL_ALT       += args.alt_offset
    PLATFORM_HOVER += args.alt_offset
    TAKEOFF_ALT    += args.alt_offset
    print(f"URI={args.uri}  model={args.model}  home_gz=({HOME_GX},{HOME_GY})", flush=True)

    # Snap the model back upright at its spawn before connecting — this
    # recovers from previous crashes / upside-down rests. Skip with --no-reset.
    if not args.no_reset:
        reset_pose(args.world, args.model, HOME_GX, HOME_GY, gz=0.3, yaw_rad=1.5708)
        time.sleep(2.0)  # let EKF settle after the teleport

    m = mavutil.mavlink_connection(args.uri)
    wait_heartbeat(m)

    # ensure we receive telemetry streams
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)

    set_mode(m, "GUIDED")
    arm(m)
    takeoff(m, TAKEOFF_ALT)
    time.sleep(2)

    # --- SLALOM through pylons (weave between x=40 and x=45) ---
    goto_gz(m,  38,  0, CRUISE_ALT, "approach slalom")
    goto_gz(m,  40, -5, CRUISE_ALT, "pylon weave 1 (left of y=-4 pylon)")
    goto_gz(m,  42,  5, CRUISE_ALT, "pylon weave 2 (right of y=4 pylon)")
    goto_gz(m,  45, -2, CRUISE_ALT, "pylon weave 3")
    goto_gz(m,  47,  2, CRUISE_ALT, "pylon weave 4")

    # --- ARCH 1 (fly through, opens along X) ---
    goto_gz(m,  50,  0, CRUISE_ALT, "arch1 approach")
    goto_gz(m,  54,  0, CRUISE_ALT, "arch1 exit")

    # --- ARCH 2 (rotated 90, opens along Y) ---
    # approach from -Y side, exit at +Y side
    goto_gz(m,  60, -4, CRUISE_ALT, "arch2 approach (-Y)")
    goto_gz(m,  60,  4, CRUISE_ALT, "arch2 exit (+Y)")
    goto_gz(m,  60,  0, CRUISE_ALT, "arch2 recenter")

    # --- LOW WALL (climb above, fly over) ---
    goto_gz(m,  63,  0, WALL_ALT,  "climb above wall")
    goto_gz(m,  67,  0, WALL_ALT,  "cross wall")

    # --- LANDING PLATFORM (hover above red pad) ---
    goto_gz(m,  70,  0, PLATFORM_HOVER, "above platform")
    time.sleep(3)

    # --- Return to home and land ---
    goto_gz(m,   0,  0, CRUISE_ALT, "return to home")
    land(m)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
