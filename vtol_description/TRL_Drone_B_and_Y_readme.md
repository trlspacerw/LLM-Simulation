# TRL Drone — Black & Yellow VTOL

Custom VTOL drone for **ROS 2 Humble + Gazebo Harmonic (gz-sim 8)**.
Fuselage painted **black**, both wings painted **yellow**.
Includes a fully-automatic takeoff → hover → land mission script with
configurable altitude.

---

## Table of Contents

1. [Model Overview](#1-model-overview)
2. [Package Structure](#2-package-structure)
3. [Dependencies](#3-dependencies)
4. [Launch in Gazebo](#4-launch-in-gazebo)
5. [Run the Automatic Mission](#5-run-the-automatic-mission)
6. [Mission Options](#6-mission-options)
7. [Anti-Flicker Design](#7-anti-flicker-design)
8. [URDF / SDF Colour Reference](#8-urdf--sdf-colour-reference)

---

## 1. Model Overview

| Property | Value |
|---|---|
| Vehicle type | Fixed-wing VTOL (quadplane) |
| Lift motors | 4 × brushless (CCW/CW alternating) |
| Pusher motor | 1 × rear pusher |
| Control surfaces | Aileron (full-span), 2 × ruddervators (V-tail) |
| Fuselage colour | **Black** `rgba(0, 0, 0, 1)` |
| Wing colour | **Yellow** `rgba(1, 1, 0, 1)` (both wings) |
| Physics | Kinematic (moved via `set_pose`; no gravity) |
| Coordinate frame | X = lateral, Y = vertical (up), Z = longitudinal (nose +Z) |

---

## 2. Package Structure

```
vtol_description/
├── urdf/
│   └── vtol.urdf                  ← robot description (colours set here)
├── models/vtol/
│   ├── model.sdf                  ← pre-converted Gazebo SDF (colours mirrored here)
│   ├── model.config
│   └── meshes/                    ← STL meshes (fuselage, wings, motors, props …)
├── meshes/                        ← same STLs for package:// URI resolution
├── launch/
│   ├── gazebo.launch.py           ← main launch file
│   ├── display.launch.py          ← RViz-only visualisation
│   └── vtol.rviz
├── worlds/
│   ├── vtol_empty.sdf             ← world used by ros2 launch (model spawned separately)
│   └── vtol_standalone.sdf        ← standalone CLI world (includes model inline)
├── scripts/
│   ├── mission.py                 ← automatic takeoff → hover → land  ← NEW
│   ├── run_mission.py             ← interactive mission (manual arm/takeoff/land)
│   ├── mavlink_controller.py      ← MAVLink vehicle emulator
│   └── mavlink_test.py            ← MAVLink GCS test client
├── TRL_Drone_B_and_Y_readme.md    ← this file
├── CMakeLists.txt
└── package.xml
```

---

## 3. Dependencies

| Dependency | Purpose |
|---|---|
| `ros-humble-desktop` | ROS 2 base |
| `gz-sim8` / Gazebo Harmonic | Simulation engine |
| `ros-humble-ros-gz-sim` | ROS ↔ Gazebo bridge |
| `ros-humble-robot-state-publisher` | TF broadcasting |
| `gz-transport13` (Python) | Direct gz transport for mission script |
| `gz-msgs10` (Python) | Protobuf message types |

Install ROS + Gazebo dependencies (if not already present):

```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-robot-state-publisher
```

---

## 4. Launch in Gazebo

```bash
# Source the workspace
source ~/ardu_ws/install/setup.bash

# Launch Gazebo with the VTOL spawned belly-down at z = 0.333 m
ros2 launch vtol_description gazebo.launch.py

# Optional flags:
ros2 launch vtol_description gazebo.launch.py gui:=false   # headless (server only)
ros2 launch vtol_description gazebo.launch.py rviz:=true   # also open RViz2
```

The launch file will:

1. Kill any leftover `gz sim` process (prevents stale-world connect issues).
2. Start a fresh Gazebo server with `vtol_empty.sdf`.
3. Open the Gazebo GUI (unless `gui:=false`).
4. Start `robot_state_publisher` with the URDF for TF/RViz support.
5. Spawn the VTOL model via `ros_gz_sim create` at `(0, 0, 0.333)` with Roll = +90° (belly-down, propellers facing up).

> **Note:** The model has `<kinematic>true</kinematic>` and gravity disabled on all structural links.  It floats at its spawn height and is repositioned by the mission script via the `set_pose` Gazebo service.

---

## 5. Run the Automatic Mission

Open a **second terminal** while Gazebo is running:

```bash
source ~/ardu_ws/install/setup.bash
cd ~/ardu_ws/install/vtol_description/share/vtol_description/scripts

# Default: climb to 5 m, hover 10 s, land
python3 mission.py

# Custom altitude
python3 mission.py --alt 15

# Custom altitude + hover duration
python3 mission.py --alt 10 --hover 30

# Custom speed (climb and descent)
python3 mission.py --alt 8 --speed 2.5
```

The script runs through **4 phases** automatically, printing live progress:

```
─────────────────────────────────────────────────────
[mission]  PHASE 1/4 — ARM  (spooling up propellers)
─────────────────────────────────────────────────────
  [spool-up]  [████████████████████████]  100%

─────────────────────────────────────────────────────
[mission]  PHASE 2/4 — TAKEOFF  → 5.0 m AGL
─────────────────────────────────────────────────────
  [takeoff]  z=5.333 m   AGL=5.00 m   (3.3 / 3.3 s)

─────────────────────────────────────────────────────
[mission]  PHASE 3/4 — HOVER  (10 s)
─────────────────────────────────────────────────────
  [hover]  z=5.333 m   AGL=5.00 m   7 s remaining

─────────────────────────────────────────────────────
[mission]  PHASE 4/4 — LAND
─────────────────────────────────────────────────────
  [land]  z=0.333 m   AGL=0.00 m   (3.3 / 3.3 s)
```

Press **Ctrl+C** at any time to abort — the script will stop the propellers and snap the model back to the ground safely.

---

## 6. Mission Options

| Argument | Default | Description |
|---|---|---|
| `--alt METRES` | `5.0` | Takeoff altitude above ground level (m) |
| `--hover SECONDS` | `10.0` | Time to hold at altitude before landing (s) |
| `--speed M_PER_S` | `1.5` | Climb and descent speed (m/s) |

**Examples:**

```bash
python3 mission.py --alt 2                    # low hover test
python3 mission.py --alt 20 --hover 60        # high altitude, 1-minute hold
python3 mission.py --alt 5 --speed 0.5        # very slow, cinematic climb
python3 mission.py --alt 10 --hover 5 --speed 4.0  # quick dash up and back
```

---

## 7. Anti-Flicker Design

Gazebo flickering in kinematic-body simulations typically has two causes:

### Cause 1 — Service latency accumulates in the ramp loop

The naive approach calls `set_pose` inside the motion loop and waits for the
service response before advancing.  If a response takes 70 ms instead of 16 ms
(one 60 Hz frame), the loop falls behind by 54 ms — and this lag compounds,
causing visible stutters.

**Fix:** The ramp loop only writes to a shared `_target_z` float.  A separate
**background pose-streaming thread** reads `_target_z` and calls `set_pose` at
60 Hz with a short 80 ms timeout.  If a call is slow it is abandoned — the
thread immediately retries with the already-updated position, so the model
catches up in one frame.

```
  Ramp thread              Pose-stream thread (daemon, 60 Hz)
  ───────────              ──────────────────────────────────
  z = interpolate()  ───▶  read _target_z
  _target_z = z            call set_pose(z, timeout=80 ms)
  sleep(1/60 s)            if slow → abandon, loop immediately
```

### Cause 2 — Ground-plane z-fighting when the model is at rest

When the model rests exactly at z = 0, the lowest geometry (propeller
underside) co-planar with the ground plane causes depth-buffer fighting,
showing as random flicker on the bottom of the model.

**Fix:** `GROUND_Z = 0.333` lifts the base-link origin 11 mm above the URDF
spawn value, giving the lowest mesh (prop visual at base_z − 0.323 m) a
guaranteed 10 mm clearance above z = 0.

Additionally, the pose thread **keeps streaming `GROUND_Z` at 10 Hz even while
the model is idle** on the ground, preventing any creep or drift that could
close that clearance gap over time.

---

## 8. URDF / SDF Colour Reference

Colours are defined in two places (both must match):

| File | Element | Colour | RGBA |
|---|---|---|---|
| `urdf/vtol.urdf` | `fuselage_mat` | Black | `0.0 0.0 0.0 1.0` |
| `urdf/vtol.urdf` | `wing_left_mat` | Yellow | `1.0 1.0 0.0 1.0` |
| `urdf/vtol.urdf` | `wing_right_mat` | Yellow | `1.0 1.0 0.0 1.0` |
| `models/vtol/model.sdf` | `base_link_visual` (diffuse/ambient) | Black | `0 0 0 1` |
| `models/vtol/model.sdf` | `wing_left_visual_7` (diffuse/ambient) | Yellow | `1 1 0 1` |
| `models/vtol/model.sdf` | `wing_right_visual_8` (diffuse/ambient) | Yellow | `1 1 0 1` |

> The URDF colours are used when the model is spawned via the `robot_description`
> topic (ros2 launch workflow).  The SDF colours are used when the model is loaded
> directly by Gazebo (standalone SDF workflow).  Keeping both in sync ensures
> consistent appearance in all launch modes.
