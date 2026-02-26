# Custom VTOL URDF Setup — `vtol_description`

Complete guide covering how the custom VTOL URDF was built from STL meshes,
how the Gazebo simulation was configured, and how to run both the interactive
`ros2 launch` workflow and the standalone CLI/MAVLink workflow.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Prerequisites](#2-prerequisites)
3. [Package structure](#3-package-structure)
4. [URDF design](#4-urdf-design)
5. [Build instructions](#5-build-instructions)
6. [Gazebo standalone model (SDF)](#6-gazebo-standalone-model-sdf)
7. [Workflow A — ros2 launch + interactive mission](#7-workflow-a--ros2-launch--interactive-mission)
8. [Workflow B — CLI Gazebo + MAVLink controller](#8-workflow-b--cli-gazebo--mavlink-controller)
9. [Script reference](#9-script-reference)
10. [Key constants and tuning](#10-key-constants-and-tuning)
11. [Issues solved and design decisions](#11-issues-solved-and-design-decisions)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Overview

`vtol_description` is a **ROS 2 / Gazebo Harmonic** package that:

- Defines a custom fixed-wing VTOL quad-plane as a **URDF** built from 17 STL
  mesh files (fuselage, wings, ailerons, V-tail ruddervators, 4 lift motors +
  props, pusher motor).
- Spawns the model in Gazebo with belly-down orientation and smooth kinematic
  motion via the `set_pose` service at 60 Hz.
- Provides **two independent control workflows**:
  - **Workflow A** — `ros2 launch` + interactive `run_mission.py`
    (type `arm`, `takeoff`, `land` in a terminal).
  - **Workflow B** — standalone `gz sim` + `mavlink_controller.py`
    (accepts real MAVLink commands over TCP, no ROS required).

---

## 2. Prerequisites

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Gazebo | Harmonic (gz-sim 8.x) |
| Python | 3.10+ |
| pymavlink | any recent (for Workflow B) |
| gz-transport13 / gz-msgs10 | included with Gazebo Harmonic |

Install ROS / Gazebo bridge packages:

```bash
sudo apt install \
  ros-humble-ros-gz-sim \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2
```

Install pymavlink (Workflow B only):

```bash
pip3 install pymavlink
```

---

## 3. Package structure

```
vtol_description/
├── CMakeLists.txt
├── package.xml
├── derick_VTOL_readme.md          ← original build notes
├── custom_VTOL_URDF_setup.md      ← this file
│
├── urdf/
│   └── vtol.urdf                  ← full robot description (17 links, joints,
│                                     Gazebo JointController plugins)
├── meshes/                        ← 17 STL files (millimetre scale)
│   ├── fuselage.stl
│   ├── wing_left.stl / wing_right.stl
│   ├── aileron.stl
│   ├── tail_left.stl / tail_right.stl
│   ├── ruddervator_left.stl / ruddervator_right.stl
│   ├── motor_1_body.stl … motor_4_body.stl
│   ├── prop_1.stl … prop_4.stl
│   └── pusher.stl
│
├── models/
│   └── vtol/                      ← standalone Gazebo model (Workflow B)
│       ├── model.config
│       ├── model.sdf              ← generated via: gz sdf -p urdf/vtol.urdf
│       └── meshes/                ← copy of ../meshes/ for self-contained model
│           └── *.stl
│
├── worlds/
│   ├── vtol_empty.sdf             ← empty world for ros2 launch (no model)
│   └── vtol_standalone.sdf       ← world WITH embedded model for CLI
│
├── launch/
│   ├── display.launch.py          ← RViz only (no Gazebo)
│   ├── gazebo.launch.py           ← Gazebo + URDF spawn (ros2 launch base)
│   └── mission.launch.py          ← Gazebo + reminder to run run_mission.py
│
└── scripts/
    ├── run_mission.py             ← interactive mission (arm/takeoff/land prompt)
    ├── mavlink_controller.py      ← MAVLink vehicle emulator (Workflow B)
    ├── mavlink_test.py            ← automated test client for Workflow B
    └── vtol_env.sh                ← environment setup for CLI workflow
```

---

## 4. URDF design

### 4.1 Coordinate convention

The STL files were exported from CAD in the following frame:

| Axis | Direction |
|------|-----------|
| **X** | Lateral — +X = right wing tip |
| **Y** | Vertical — +Y = up (top of fuselage) |
| **Z** | Longitudinal — +Z = nose, −Z = tail |

Gazebo uses Z-up (REP-103).  To correct the mismatch the model is spawned
with **Roll = +90°** (π/2 rad around world X), mapping model +Y → world +Z.

### 4.2 Mesh scale

All STL files are in **millimetres**.  Every `<mesh>` tag in the URDF uses:

```xml
<mesh filename="package://vtol_description/meshes/fuselage.stl"
      scale="0.001 0.001 0.001"/>
```

### 4.3 Link tree

```
base_link  (fuselage — kinematic, gravity off)
├── wing_left_joint        [fixed]    → wing_left_link
├── wing_right_joint       [fixed]    → wing_right_link
├── aileron_link_joint     [revolute, X-axis ±0.44 rad]  → aileron_link
├── tail_left_link_joint   [fixed]    → tail_left_link
├── tail_right_link_joint  [fixed]    → tail_right_link
├── ruddervator_left_link_joint   [revolute, +45° diagonal ±0.44 rad]
├── ruddervator_right_link_joint  [revolute, −45° diagonal ±0.44 rad]
├── motor_1_joint  [fixed] → motor_1_link (kinematic)
│   └── prop_1_joint  [continuous, Y-axis]  → prop_1_link  ← dynamic, JointController
├── motor_2_joint  [fixed] → motor_2_link (kinematic)
│   └── prop_2_joint  [continuous, Y-axis]  → prop_2_link  ← dynamic, JointController
├── motor_3_joint  [fixed] → motor_3_link (kinematic)
│   └── prop_3_joint  [continuous, Y-axis]  → prop_3_link  ← dynamic, JointController
├── motor_4_joint  [fixed] → motor_4_link (kinematic)
│   └── prop_4_joint  [continuous, Y-axis]  → prop_4_link  ← dynamic, JointController
└── pusher_joint   [continuous, Z-axis]     → pusher_link  ← dynamic, JointController
```

### 4.4 Physics strategy

| Link category | `<kinematic>` | `<gravity>` | Controlled by |
|---------------|---------------|-------------|---------------|
| base_link, wings, tail, aileron, ruddervators, motor bodies | `1` (kinematic) | `0` | `set_pose` service |
| prop_1–4_link, pusher_link | `0` (dynamic) | `0` | `JointController` plugin |

**Why this split:**  The whole airframe is moved as a rigid kinematic body
via repeated `set_pose` calls (smooth, no physics drift).  Only the 5 prop
joints are left dynamic so the `JointController` plugin can spin them by
applying a velocity command.

### 4.5 JointController plugins (in `vtol.urdf`)

Five `<gazebo>` plugins are appended to the URDF — one per spinning joint.
All start at `initial_velocity = 0.0` (static at spawn):

```xml
<gazebo>
  <plugin filename="gz-sim-joint-controller-system"
          name="gz::sim::systems::JointController">
    <joint_name>prop_1_joint</joint_name>
    <initial_velocity>0.0</initial_velocity>
  </plugin>
</gazebo>
```

Each plugin subscribes to:
```
/model/vtol/joint/<joint_name>/cmd_vel   (gz.msgs10.Double)
```

| Joint | Direction | Full speed (rad/s) |
|-------|-----------|--------------------|
| prop_1_joint | CCW | +50 |
| prop_2_joint | CW  | −50 |
| prop_3_joint | CCW | +50 |
| prop_4_joint | CW  | −50 |
| pusher_joint | CCW | +30 |

---

## 5. Build instructions

```bash
cd ~/ardu_ws
colcon build --packages-select vtol_description
source install/setup.bash
```

To rebuild after any change to URDF, scripts, worlds, or models:

```bash
colcon build --packages-select vtol_description && source install/setup.bash
```

---

## 6. Gazebo standalone model (SDF)

The `models/vtol/` directory contains a **self-contained Gazebo model** used
by Workflow B (CLI).  It was generated once from the URDF:

```bash
gz sdf -p src/vtol_description/urdf/vtol.urdf > \
        src/vtol_description/models/vtol/model.sdf
```

After conversion, mesh URIs in `model.sdf` were updated from
`model://vtol_description/meshes/` to `model://vtol/meshes/` and the mesh
files were copied into `models/vtol/meshes/` to make the model self-contained.

The `models/vtol/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>VTOL Fixed-Wing QuadPlane</name>
  <version>1.0</version>
  <sdf version="1.11">model.sdf</sdf>
</model>
```

---

## 7. Workflow A — ros2 launch + interactive mission

This workflow spawns the VTOL via `ros_gz_sim create` (from the URDF) and
provides an interactive terminal prompt to arm, take off, and land.

### 7.1 Launch Gazebo + model

```bash
source ~/ardu_ws/install/setup.bash
ros2 launch vtol_description mission.launch.py
```

What happens internally:
1. Any leftover `gz sim` processes are killed.
2. Gazebo server starts with `vtol_empty.sdf` (empty world, no model).
3. Gazebo GUI opens.
4. The VTOL URDF is spawned at `(x=0, y=0, z=0.333, Roll=90°)`.
5. A reminder is printed after 8 s to start `run_mission.py`.

Optional arguments:

```bash
ros2 launch vtol_description mission.launch.py gui:=false   # headless
ros2 launch vtol_description gazebo.launch.py rviz:=true    # + RViz
```

### 7.2 Run the interactive mission

In a **second terminal** (after Gazebo is up):

```bash
source ~/ardu_ws/install/setup.bash
python3 ~/ardu_ws/install/vtol_description/share/vtol_description/scripts/run_mission.py
```

At the prompt type commands one at a time:

```
> arm          # props spool up over 1 s
> takeoff      # ascend to 1 m AGL over 2 s
> land         # descend and spool down after 2.5 s touch-down wait
> quit         # exit (props stop, model returns to ground)
```

Full terminal session example:

```
[mission] Model ready.

┌─────────────────────────────────────────┐
│  STATE: IDLE  (propellers stopped)       │
└─────────────────────────────────────────┘
  Commands: "arm" | "quit"

  > arm

[mission] ▶ ARMING — spooling up propellers ...
  [spool-up]  [████████████████████]  100%   (1.0/1.0 s)

┌─────────────────────────────────────────┐
│  STATE: ARMED  (propellers spinning)     │
└─────────────────────────────────────────┘
  Commands: "takeoff" | "disarm" | "quit"

  > takeoff

[mission] ▲ TAKE-OFF → 1.33 m
  [take-off]  0.333 m → 1.333 m   (2.0/2.0 s)

  > land

[mission] ▼ LANDING → ground
  [land]  1.333 m → 0.333 m   (2.0/2.0 s)
[mission] ✓ Touchdown!  Spooling down in 2.5 s ...
```

---

## 8. Workflow B — CLI Gazebo + MAVLink controller

This workflow runs without ROS.  Gazebo loads the world directly with the
model already embedded, and `mavlink_controller.py` acts as an ArduPilot
vehicle listening on TCP port 5760.

### 8.1 Set up the environment (once per shell)

```bash
source ~/ardu_ws/install/vtol_description/share/vtol_description/scripts/vtol_env.sh
```

This sets `GZ_SIM_RESOURCE_PATH` and exports `$VTOL_WORLDS_DIR` and
`$VTOL_SCRIPTS_DIR` for convenience.

### 8.2 Terminal 1 — start Gazebo

```bash
gz sim -v4 -r "$VTOL_WORLDS_DIR/vtol_standalone.sdf"
```

The VTOL model appears immediately at `(0, 0, 0.333 m)`, props static.

### 8.3 Terminal 2 — start the MAVLink controller

```bash
python3 "$VTOL_SCRIPTS_DIR/mavlink_controller.py"
```

Expected output:

```
[gz] Gazebo connected.
[mav] Opening MAVLink on tcpin:0.0.0.0:5760 ...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  MAVLink vehicle ready on tcpin:0.0.0.0:5760
  Connect Mission Planner  →  TCP  127.0.0.1 : 5760
  Or run:  python3 mavlink_test.py --connect tcp:127.0.0.1:5760
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 8.4 Terminal 3 — send MAVLink commands

**Option A — automated test (arm → takeoff 10 m → hover 5 s → land):**

```bash
python3 "$VTOL_SCRIPTS_DIR/mavlink_test.py" --connect tcp:127.0.0.1:5760
```

**Option B — Mission Planner / QGroundControl:**

Connect to `TCP 127.0.0.1 : 5760`.  The vehicle appears as
`MAV_TYPE_VTOL_QUADROTOR`, autopilot `ARDUPILOTMEGA`, always in GUIDED mode.

**Option C — mavproxy.py:**

```bash
mavproxy.py --master=tcp:127.0.0.1:5760
# Then:
mode GUIDED
arm throttle
takeoff 10
```

### 8.5 MAVLink command sequence

The controller accepts the following standard MAVLink commands in order:

| Step | Command | Effect in Gazebo |
|------|---------|-----------------|
| 1 | `MAV_CMD_DO_SET_MODE` (GUIDED) | Always accepted |
| 2 | `MAV_CMD_COMPONENT_ARM_DISARM` (param1=1) | Props spool up over 1 s |
| 3 | `MAV_CMD_NAV_VTOL_TAKEOFF` (param7 = alt AGL) | Ascend at 1.5 m/s |
| 4 | `MAV_CMD_NAV_VTOL_LAND` | Descend at 1.5 m/s, props stop 2.5 s after touchdown |

### 8.6 Telemetry sent back to GCS

| Message | Rate | Content |
|---------|------|---------|
| `HEARTBEAT` | 1 Hz | type=VTOL_QUADROTOR, autopilot=ARDUPILOTMEGA |
| `GLOBAL_POSITION_INT` | 5 Hz | relative_alt in mm (AGL from GROUND_Z) |
| `EXTENDED_SYS_STATE` | 2 Hz | landed_state (ON_GROUND / TAKEOFF / IN_AIR / LANDING) |
| `COMMAND_ACK` | on demand | result for every COMMAND_LONG received |

---

## 9. Script reference

### `run_mission.py` — interactive mission (Workflow A)

| Constant | Default | Description |
|----------|---------|-------------|
| `GROUND_Z` | `0.333` | World Z of base_link at rest |
| `HOVER_Z` | `GROUND_Z + 1.0` | Hover altitude (AGL + GROUND_Z) |
| `POSE_RATE_HZ` | `60` | set_pose calls per second during ramps |
| `SPOOL_UP_S` | `1.0` | Prop spin-up duration (s) |
| `SPOOL_DOWN_S` | `2.5` | Touchdown-to-prop-stop delay (s) |
| `TAKEOFF_S` | `2.0` | Duration of takeoff ramp |
| `LAND_S` | `2.0` | Duration of landing ramp |

To change hover altitude, edit `HOVER_Z`:

```python
HOVER_Z = GROUND_Z + 5.0   # hover at 5 m AGL
```

### `mavlink_controller.py` — MAVLink vehicle emulator (Workflow B)

| Constant | Default | Description |
|----------|---------|-------------|
| `GROUND_Z` | `0.333` | World Z of base_link at rest |
| `DEFAULT_ALT_M` | `10.0` | Default takeoff altitude if param7 = 0 |
| `POSE_RATE_HZ` | `60` | set_pose calls per second |
| `CLIMB_SPEED` | `1.5` | Ascent / descent speed (m/s) |
| `SPOOL_UP_S` | `1.0` | Prop spin-up ramp duration (s) |
| `SPOOL_DOWN_WAIT_S` | `2.5` | Post-touchdown wait before spool-down |

Launch with a different port or transport:

```bash
python3 mavlink_controller.py --listen udpin:0.0.0.0:14550
python3 mavlink_controller.py --listen tcpin:0.0.0.0:5760   # default
```

### `mavlink_test.py` — automated GCS test

```bash
python3 mavlink_test.py --connect tcp:127.0.0.1:5760   # default
python3 mavlink_test.py --connect udpout:127.0.0.1:14550
```

---

## 10. Key constants and tuning

### `GROUND_Z = 0.333` m

This is the world-Z coordinate of `base_link` when the model is resting on
the ground plane (z = 0).

It was set to **0.333 m** (11 mm above the raw URDF bounding-box value of
0.3223 m) to ensure all prop visual meshes clear z = 0.  The prop visual
offsets in the converted SDF place the propeller disk frame at approximately
`world_z = base_z − 0.323 m`.  At `base_z = 0.3223` that equals `−0.001 m`
(just below the ground plane), causing z-fighting and visual flickering.
Raising `GROUND_Z` to `0.333` gives a 10 mm clearance.

Both `run_mission.py`, `mavlink_controller.py`, the spawn argument in
`gazebo.launch.py`, and the `<pose>` in `vtol_standalone.sdf` must all use
the **same** value.

### Pose update rate (60 Hz)

The scripts use `gz.transport13` Python bindings for direct IPC calls to the
`/world/vtol_world/set_pose` service.  Each call takes ~1–2 ms, achieving a
smooth 60 Hz update rate.  The previous `gz service` subprocess approach gave
~8 Hz with visible stepping.

### Prop velocity (rad/s)

Lift props: ±50 rad/s (~478 RPM).  Pusher: +30 rad/s (~287 RPM).  Adjust in
`PROP_JOINTS` list inside each script.

---

## 11. Issues solved and design decisions

### Gazebo GUI connecting to wrong world

**Problem:** When a previous Gazebo session left a server running, the new GUI
connected to the old server (e.g. `quadplane_tracking` world) instead of the
new `vtol_world`.

**Fix:** `gazebo.launch.py` runs `pkill -9 -f "gz sim"` before starting the
server, with a 2-second delay to let the kill complete.

### Steppy / 1-fps movement

**Problem:** Using `subprocess.run(['gz', 'service', ...])` for each `set_pose`
call took ~120–200 ms per call, limiting updates to ~8 Hz.  Movement looked
like stop-motion animation.

**Fix:** Replaced with `gz.transport13` Python bindings (`node.request()`),
which take ~1–2 ms per call.  Rate raised to 60 Hz.

### Propellers not spinning

**Problem:** All links had `<kinematic>1</kinematic>` in the URDF.  The
`JointController` plugin uses force/torque commands which are ignored on
kinematic bodies.

**Fix:** Removed `<kinematic>` from the 5 prop/pusher links only.  These
remain dynamic (JointController works) while the airframe remains kinematic
(set_pose works).  Gravity is disabled on all links.

### VTOL not taking off (state machine race condition)

**Problem:** The GCS sent ARM immediately followed by VTOL_TAKEOFF.  The
flight thread received ARM, then cleared `_cmd_takeoff` during its startup
sequence.  By the time it checked for a takeoff command, the event had been
wiped.

**Fix:** Removed the `_cmd_takeoff.clear()` from the ARM startup block.  The
event is only cleared when the flight thread is actually waiting for it in the
inner loop.  Additionally, the MAVLink receive thread pre-sets
`_state = ST_ARMED` before calling `_cmd_arm.set()` so a fast VTOL_TAKEOFF
arriving before the flight thread wakes up is not rejected.

### VTOL appearing and disappearing (z-fighting)

**Problem:** The prop visual mesh offsets in the converted SDF placed the
propeller disk frames at world z ≈ −0.001 m (1 mm below the ground plane at
z = 0).  The renderer alternated between showing the mesh and the ground
surface — classic z-fighting.

**Fix:** Raised `GROUND_Z` from `0.3223` to `0.333` m so prop visuals sit
at z ≈ +0.010 m, cleanly above the ground plane.

### ros2 launch double-model conflict

**Problem:** After adding the `<include>` block to `vtol_empty.sdf` for the
CLI workflow, the `ros2 launch` workflow loaded a world that already contained
a model named "vtol", then tried to spawn a second "vtol" via
`ros_gz_sim create` — causing a naming conflict.

**Fix:** Split into two world files:
- `vtol_empty.sdf` — no model (used by `gazebo.launch.py`, model spawned separately).
- `vtol_standalone.sdf` — model embedded (used by CLI `gz sim` command).

### MAVLink TCP vs UDP deadlock

**Problem:** Using `udpin:0.0.0.0:14550` blocked in `wait_heartbeat()` before
threads started.  Neither side sent heartbeats first.

**Fix:** Switched to `tcpin:0.0.0.0:5760` (server/listen mode like ArduPilot
primary port) and start all threads immediately before any blocking call.

---

## 12. Troubleshooting

### "Model does not appear" / mesh errors

Gazebo cannot find mesh files → check that `GZ_SIM_RESOURCE_PATH` includes
the models directory.

**Workflow A:** handled automatically by `gazebo.launch.py`.

**Workflow B:** source the env file first:

```bash
source ~/ardu_ws/install/vtol_description/share/vtol_description/scripts/vtol_env.sh
```

Or set manually:

```bash
export GZ_SIM_RESOURCE_PATH=\
  ~/ardu_ws/install/vtol_description/share/vtol_description/models:\
  $GZ_SIM_RESOURCE_PATH
```

### "Address already in use" on port 5760

A previous `mavlink_controller.py` process is still holding the port:

```bash
fuser -k 5760/tcp
```

### "set_pose service not reachable"

Gazebo is not running or the world name is wrong.  Check:

```bash
gz service --list | grep set_pose
# Expected: /world/vtol_world/set_pose
```

### Propellers not spinning after ARM

The `JointController` topics must match exactly.  Verify with:

```bash
gz topic --list | grep cmd_vel
# Expected (5 lines):
# /model/vtol/joint/prop_1_joint/cmd_vel
# /model/vtol/joint/prop_2_joint/cmd_vel
# /model/vtol/joint/prop_3_joint/cmd_vel
# /model/vtol/joint/prop_4_joint/cmd_vel
# /model/vtol/joint/pusher_joint/cmd_vel
```

If these topics are absent, the JointController plugins did not load.  Confirm
that `model.sdf` (Workflow B) or `vtol.urdf` (Workflow A) contains all five
`<plugin filename="gz-sim-joint-controller-system" ...>` blocks.

### Drone doesn't move after VTOL_TAKEOFF

The controller must receive ARM **before** VTOL_TAKEOFF.  Confirm the
command sequence and check the controller terminal for state messages.

If the controller prints `VTOL_TAKEOFF rejected — state is DISARMED`, the ARM
ACK was lost or not sent.  Retry the full sequence: SET_MODE → ARM → TAKEOFF.

### Model still flickering after GROUND_Z fix

Verify both the world SDF and the controller script use the same value:

```bash
grep "0.333" \
  ~/ardu_ws/install/vtol_description/share/vtol_description/worlds/vtol_standalone.sdf \
  ~/ardu_ws/install/vtol_description/share/vtol_description/scripts/mavlink_controller.py \
  ~/ardu_ws/install/vtol_description/share/vtol_description/scripts/run_mission.py \
  ~/ardu_ws/install/vtol_description/share/vtol_description/launch/gazebo.launch.py
```

All four files should contain `0.333`.

### RViz only (no Gazebo)

```bash
ros2 launch vtol_description display.launch.py
```

This opens RViz with the URDF rendered from `robot_description` topic.
Use the joint_state_publisher_gui sliders to move ailerons, ruddervators, and
see prop joint positions.

---

*Package: `vtol_description` | ROS 2 Humble | Gazebo Harmonic 8.x*
