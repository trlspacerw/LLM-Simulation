# derick_VTOL_readme.md
# Custom VTOL — `vtol_description` Package

A ROS 2 / Gazebo Harmonic package that loads a custom fixed-wing VTOL model
from STL meshes, spawns it in simulation, and runs a simple demo mission.

---

## Table of Contents

1. [What this package does](#1-what-this-package-does)
2. [Package structure](#2-package-structure)
3. [URDF design](#3-urdf-design)
4. [Build instructions](#4-build-instructions)
5. [Launch files](#5-launch-files)
6. [Running the simulation](#6-running-the-simulation)
7. [How to modify the physics](#7-how-to-modify-the-physics)
8. [Mission script explained](#8-mission-script-explained)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. What this package does

- Defines the VTOL vehicle as a **URDF** (Unified Robot Description Format) built
  from 19 STL mesh files (fuselage, wings, ailerons, tail fins, ruddervators,
  4 lift motors + props, pusher motor).
- Spawns the model **belly-down** in an empty Gazebo world.
- Provides a **demo mission script** that takes off to 1 m, hovers for 5 s,
  then lands — all without ArduPilot SITL (pure kinematic animation via the
  Gazebo `set_pose` service).

---

## 2. Package structure

```
vtol_description/
├── CMakeLists.txt
├── package.xml
├── derick_VTOL_readme.md       ← this file
│
├── urdf/
│   └── vtol.urdf               ← full robot description (17 links, joints,
│                                  Gazebo physics tags)
├── meshes/
│   ├── fuselage.stl
│   ├── wing_left.stl / wing_right.stl
│   ├── aileron.stl
│   ├── tail_left.stl / tail_right.stl
│   ├── ruddervator_left.stl / ruddervator_right.stl
│   ├── motor_1_body.stl … motor_4_body.stl
│   ├── prop_1.stl … prop_4.stl
│   └── pusher.stl
│
├── worlds/
│   └── vtol_empty.sdf          ← minimal Gazebo world (ground plane + sun)
│
├── launch/
│   ├── display.launch.py       ← RViz only (no Gazebo, no physics)
│   ├── gazebo.launch.py        ← Gazebo only (model spawned, no mission)
│   └── mission.launch.py       ← Gazebo + auto demo mission
│
└── scripts/
    └── run_mission.py          ← standalone mission script
```

---

## 3. URDF design

### 3.1 Coordinate convention (STL export frame)

| Axis | Direction |
|------|-----------|
| **X** | Lateral — +X = right wing tip |
| **Y** | Vertical — +Y = up (top of fuselage) |
| **Z** | Longitudinal — +Z = nose, −Z = tail |

> **Important for Gazebo:** Gazebo uses Z-up / X-forward (ROS REP-103).
> To correct for the mismatch the model is **spawned with Roll = +90°**
> so the URDF's +Y becomes the world's +Z (up).

### 3.2 Mesh scale

All STL files were exported in **millimetres**.
Every `<mesh>` tag uses `scale="0.001 0.001 0.001"` to convert to metres.

### 3.3 Link tree

```
base_link  (fuselage — origin at bounding-box centre / CG)
├── wing_left_joint     [fixed]  → wing_left
├── wing_right_joint    [fixed]  → wing_right
├── aileron_link_joint  [revolute, X-axis ±0.44 rad]  → aileron_link
├── tail_left_link_joint  [fixed] → tail_left_link
├── tail_right_link_joint [fixed] → tail_right_link
├── ruddervator_left_link_joint  [revolute, +45° diagonal ±0.44 rad]
├── ruddervator_right_link_joint [revolute, −45° diagonal ±0.44 rad]
├── motor_1_joint  [fixed] → motor_1_link
│   └── prop_1_joint  [continuous, Y-axis]  → prop_1_link
├── motor_2_joint  [fixed] → motor_2_link
│   └── prop_2_joint  [continuous, Y-axis]  → prop_2_link
├── motor_3_joint  [fixed] → motor_3_link
│   └── prop_3_joint  [continuous, Y-axis]  → prop_3_link
├── motor_4_joint  [fixed] → motor_4_link
│   └── prop_4_joint  [continuous, Y-axis]  → prop_4_link
└── pusher_joint   [continuous, Z-axis]  → pusher_link
```

---

## 4. Build instructions

```bash
cd ~/ardu_ws
colcon build --packages-select vtol_description
source install/setup.bash
```

---

## 5. Launch files

| Launch file | What it starts |
|-------------|---------------|
| `display.launch.py` | RViz2 + robot_state_publisher (no Gazebo) |
| `gazebo.launch.py` | Gazebo server + GUI + model spawn |
| `mission.launch.py` | Gazebo + model spawn + auto demo mission |

### Arguments

```bash
# Gazebo headless (server only, no GUI window):
ros2 launch vtol_description gazebo.launch.py gui:=false

# Gazebo + RViz side by side:
ros2 launch vtol_description gazebo.launch.py rviz:=true

# Mission without GUI (for logging / CI):
ros2 launch vtol_description mission.launch.py gui:=false
```

---

## 6. Running the simulation

### Step 1 — Source the workspace

```bash
source ~/ardu_ws/install/setup.bash
```

### Step 2a — View only in Gazebo

```bash
ros2 launch vtol_description gazebo.launch.py
```

The drone spawns **belly-down on the ground** at position (0, 0, 0.3223 m).
It floats in place because gravity and physics are disabled (see Section 7).

### Step 2b — Automatic demo mission

```bash
ros2 launch vtol_description mission.launch.py
```

Timeline after launch:

| Time | Event |
|------|-------|
| 0 s | Gazebo server starts |
| ~1 s | Gazebo GUI opens |
| 3 s | VTOL model spawns |
| 8 s | Mission script starts |
| 13 s | Take-off begins (z: 0.32 → 1.32 m over ~2 s) |
| 15 s | Hover at 1.32 m for 5 s |
| 20 s | Land (z: 1.32 → 0.32 m over ~2 s) |
| 22 s | Mission complete — drone rests on ground |

### Step 2c — Run the mission again without restarting

While Gazebo is open from any of the above:

```bash
python3 ~/ardu_ws/install/vtol_description/share/vtol_description/scripts/run_mission.py
```

### Step 3 — View in RViz only (no Gazebo)

```bash
ros2 launch vtol_description display.launch.py
```

---

## 7. How to modify the physics

All physics configuration lives in two places:

| What | Where |
|------|-------|
| Per-link physics tags | End of `urdf/vtol.urdf` (the `<gazebo reference="...">` blocks) |
| World physics | `worlds/vtol_empty.sdf` (the `<physics>` element) |

### 7.1 Enable / disable gravity

**Current state:** gravity is OFF for all 17 links (model floats).

To **re-enable gravity** for a link, find its `<gazebo>` block at the bottom
of `urdf/vtol.urdf` and change `<gravity>0</gravity>` to `<gravity>1</gravity>`:

```xml
<!-- BEFORE (gravity off) -->
<gazebo reference="base_link"><gravity>0</gravity><kinematic>1</kinematic></gazebo>

<!-- AFTER (gravity on) -->
<gazebo reference="base_link"><gravity>1</gravity><kinematic>0</kinematic></gazebo>
```

> If you re-enable gravity you **must** also change `<kinematic>` (see 7.2)
> and replace box collision shapes with realistic ones (see 7.3), otherwise
> the model will fall through the ground.

### 7.2 Kinematic vs dynamic mode

**Kinematic** (`<kinematic>1</kinematic>`):
- The link is moved **only** by external position commands (`set_pose`).
- The physics engine **ignores forces, gravity, and collisions** on this link.
- No velocity accumulation → **stable, no oscillation**.
- Used for the current demo mission.

**Dynamic** (`<kinematic>0</kinematic>`):
- Normal physics: gravity, collisions, and applied forces all affect the link.
- Required for realistic flight simulation with ArduPilot SITL.
- Requires working collision shapes (see 7.3).

Change in `urdf/vtol.urdf`:

```xml
<!-- kinematic (demo) -->
<gazebo reference="base_link">
  <gravity>0</gravity>
  <kinematic>1</kinematic>
</gazebo>

<!-- dynamic (physics simulation) -->
<gazebo reference="base_link">
  <gravity>1</gravity>
  <kinematic>0</kinematic>
</gazebo>
```

After editing, always rebuild:

```bash
colcon build --packages-select vtol_description
```

### 7.3 Collision shapes

Gazebo Harmonic's default physics engine (**dartsim**) does **not** support
mesh-based collision shapes loaded from URDF.  Using `<mesh>` inside a
`<collision>` block causes silent failures and the model falls through the
ground.

**Current state:** every `<collision>` block uses a placeholder 0.1 m box:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>    <!-- placeholder -->
  </geometry>
</collision>
```

To get realistic collision (e.g. for ground interaction), replace the box with
a properly-sized primitive.  Example for the fuselage:

```xml
<!-- Fuselage: ~3.25 m wide × 0.64 m tall × 2.04 m long -->
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="3.25 0.64 2.04"/>
  </geometry>
</collision>
```

Other useful primitives:

```xml
<!-- Cylinder (good for motor bodies and props) -->
<geometry>
  <cylinder radius="0.15" length="0.08"/>
</geometry>

<!-- Sphere (good for small rounded parts) -->
<geometry>
  <sphere radius="0.05"/>
</geometry>
```

> **Note:** if you switch to realistic collision shapes you must also
> set `<gravity>1</gravity>` and `<kinematic>0</kinematic>` so the model
> actually rests on the collision surface.

### 7.4 Mass and inertia

Mass and inertia tensors are set per-link inside the `<inertial>` block in
`urdf/vtol.urdf`.  Current values are rough box approximations:

```xml
<link name="base_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="4.5"/>                          <!-- kg -->
    <inertia
      ixx="1.6084" ixy="0" ixz="0"
                   iyy="1.5301" iyz="0"
                                izz="0.2308"/>   <!-- kg·m² -->
  </inertial>
  ...
```

To update with measured values, replace `mass value` and the six `ixx iyy izz
ixy ixz iyz` components.  For a symmetric fuselage, `ixy = ixz = iyz = 0`.

Use the box inertia formula as a starting point:

```
Ixx = (1/12) * m * (y² + z²)   y = height, z = length
Iyy = (1/12) * m * (x² + z²)   x = width
Izz = (1/12) * m * (x² + y²)
```

### 7.5 World physics (simulation step size and speed)

Edit `worlds/vtol_empty.sdf`:

```xml
<physics name="1ms" type="ignore">
  <max_step_size>0.001</max_step_size>   <!-- simulation dt in seconds -->
  <real_time_factor>1.0</real_time_factor> <!-- 1.0 = real time, 2.0 = 2× faster -->
</physics>
```

| Parameter | Effect |
|-----------|--------|
| `max_step_size` | Smaller = more accurate but slower. Default: 0.001 s (1 kHz) |
| `real_time_factor` | >1 speeds up simulation; <1 slows it down |

### 7.6 Switching to bullet physics (supports mesh collision)

If you need mesh-accurate collision shapes, switch the physics engine from
dartsim to bullet by adding `type="bullet"` to the world's `<physics>` tag:

```xml
<physics name="bullet_1ms" type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

With bullet you can restore the original `<mesh>` collision geometry in
`vtol.urdf` and the model will collide correctly.

---

## 8. Mission script explained

`scripts/run_mission.py` controls the drone by repeatedly calling Gazebo's
`/world/vtol_world/set_pose` service via the `gz service` CLI.

### Key constants

```python
WORLD    = 'vtol_world'    # must match the world name in vtol_empty.sdf
MODEL    = 'vtol'          # must match the -name argument in the spawn call
GROUND_Z = 0.3223          # base_link z when belly is on the ground plane
HOVER_Z  = GROUND_Z + 1.0  # 1 m above ground

# Orientation quaternion for Roll = +90° (belly-down spawn)
# w=cos(π/4), x=sin(π/4), y=0, z=0
QUAT_W, QUAT_X, QUAT_Y, QUAT_Z = 0.7071, 0.7071, 0.0, 0.0

RATE_HZ = 8   # how often set_pose is called during motion phases
```

### How to change the mission profile

| What to change | Where |
|----------------|-------|
| Hover altitude | Change `HOVER_Z = GROUND_Z + 1.0` (add desired metres) |
| Hover duration | Change `duration=5.0` in the `hold(...)` call |
| Take-off / land speed | Change `duration=2.0` in the `fly(...)` calls |
| Call rate | Change `RATE_HZ` (higher = smoother, but more CPU) |

Example — hover at 3 m for 10 seconds:

```python
HOVER_Z = GROUND_Z + 3.0

...
hold('hover', HOVER_Z, duration=10.0)
```

### How `set_pose` works

```
Python script
    │
    └─► subprocess: gz service -s /world/vtol_world/set_pose
                        --reqtype gz.msgs.Pose
                        --reptype gz.msgs.Boolean
                        --req 'name:"vtol" position:{z:1.32} orientation:{...}'
                        │
                        └─► Gazebo UserCommandsSystem
                                │
                                └─► ECM: WorldPoseCmd component
                                        │
                                        └─► dartsim: moves kinematic body
                                                     (no velocity, no force)
```

---

## 9. Troubleshooting

### Model does not appear in Gazebo

**Cause:** Mesh collision shapes (`<mesh>` in `<collision>`) fail with dartsim,
giving the model no collision geometry.  Without collision, a dynamic model
falls through the ground immediately.

**Fix already applied:** all collision shapes are boxes; all links are
kinematic with gravity off.  If you see this again, verify the `<gazebo>`
tags at the bottom of `urdf/vtol.urdf` are present:

```xml
<gazebo reference="base_link"><gravity>0</gravity><kinematic>1</kinematic></gazebo>
```

### Drone oscillates up and down

**Cause:** dartsim accumulates velocity from consecutive `set_pose` teleports.
Even with gravity off, the position deltas are interpreted as velocity, causing
overshoot and bounce.

**Fix already applied:** `<kinematic>1</kinematic>` tells dartsim to move the
body by position command only — no velocity integration.

To reproduce the bug (for testing), change `<kinematic>1</kinematic>` to
`<kinematic>0</kinematic>` and rebuild.

### STL meshes not found by Gazebo (`model://vtol_description/...`)

Gazebo translates URDF `package://` URIs to `model://` URIs.  The install
share directory must be on `GZ_SIM_RESOURCE_PATH`.

`gazebo.launch.py` sets this automatically at launch time:

```python
gz_resource_path = os.path.join(get_package_prefix('vtol_description'), 'share')
os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + gz_resource_path
```

If launching Gazebo manually (not via the launch file), set it first:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ardu_ws/install/vtol_description/share
```

### Mission script: "could not reach set_pose service"

The Gazebo server must be running and the world must be named `vtol_world`.
Check with:

```bash
gz service -l | grep set_pose
# Expected: /world/vtol_world/set_pose
```

### joint_state_publisher_gui (interactive joint sliders) not installed

```bash
sudo apt install ros-humble-joint-state-publisher-gui
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

---

*Package: `vtol_description` | ROS 2 Humble | Gazebo Harmonic 8.x*
