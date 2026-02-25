# drone_nl_control

Natural language drone control for ArduPilot SITL + Gazebo Harmonic, implementing the
[Taking Flight with Dialogue](https://arxiv.org/abs/2404.04781) algorithm.

A mission description such as _"find the walking person"_ is all the drone needs. An LLM
generates `Turn(θ)` / `Move(d)` commands step-by-step while a VLM watches the front
camera and confirms target detection.

---

## Table of Contents

1. [Architecture](#architecture)
2. [Requirements](#requirements)
3. [One-Time Setup](#one-time-setup)
   - [Install Ollama and pull models](#1-install-ollama-and-pull-models)
   - [Build the ROS2 package](#2-build-the-ros2-package)
4. [Running a Single Mission](#running-a-single-mission)
5. [Running the Full Benchmark](#running-the-full-benchmark)
   - [Start the simulation](#step-1-start-the-simulation)
   - [Stage 1 — LLM command validity](#step-2-stage-1--llm-command-validity)
   - [Stage 2 — VLM binary-response validity](#step-3-stage-2--vlm-binary-response-validity)
   - [Stage 3 — Mission success rate](#step-4-stage-3--mission-success-rate)
   - [Generate the report](#step-5-generate-the-report)
   - [One-shot convenience script](#one-shot-convenience-script)
6. [Monitoring Progress](#monitoring-progress)
7. [Configuration Reference](#configuration-reference)
8. [Package Structure](#package-structure)
9. [Known Issues and Fixes](#known-issues-and-fixes)
10. [Troubleshooting](#troubleshooting)
11. [Citation](#citation)

---

## Architecture

```
Natural-language mission
        │
        ▼
  MissionController (ROS2 node)
        │
        ├─── LLMAgent ──► Ollama  (gemma3:4b / qwen2.5:3b / llama3.2:3b / deepseek-llm:7b)
        │                  Generates Turn(θ) / Move(d) commands
        │
        ├─── VLMAgent ──► Ollama  (gemma3:4b / llama3.2-vision:11b / llava:7b)
        │    │             Binary Yes/No detection from camera frame
        │    └── /front_camera/image  (ROS2 topic, 640×480 @ 15 Hz)
        │
        └─── DroneInterface ──► pymavlink ──► MAVProxy ──► ArduPilot SITL
                                              udp:127.0.0.1:14550
```

**Decision loop (up to `max_steps` steps):**

1. Query VLM: "Is `<target>` visible in this image?" → Yes / No
2. Two consecutive detections → **SUCCESS**
3. Query LLM with drone state + VLM result + command history → `Turn(θ)` or `Move(d)`
4. Execute the command via MAVLink body-velocity commands
5. Repeat until success or step limit

---

## Requirements

| Component | Version |
|-----------|---------|
| ROS2 | Humble |
| Gazebo | Harmonic (gz-sim 8.x) |
| ArduPilot | SITL V4.7.0-dev |
| ardupilot_gz | (provides `iris_tracking` launch) |
| Python | 3.10 |
| Ollama | 0.16.3+ |
| GPU | NVIDIA with ≥ 4 GB VRAM (T1000 8 GB confirmed) |

Python packages (already in the workspace): `pymavlink`, `opencv-python`, `cv_bridge`,
`requests`, `pyyaml`.

### Models used in the benchmark

| Role | Model | Size | Notes |
|------|-------|------|-------|
| LLM | `gemma3:4b` | ~3.3 GB | Also used as VLM |
| LLM | `qwen2.5:3b` | ~2 GB | |
| LLM | `llama3.2:3b` | ~2 GB | |
| LLM | `deepseek-llm:7b` | ~4.7 GB | |
| VLM | `gemma3:4b` | ~3.3 GB | Multimodal; same binary as above |
| VLM | `llama3.2-vision:11b` | ~8.7 GB | Runs split GPU/CPU on 8 GB VRAM |
| VLM | `llava:7b` | ~4.7 GB | **Must run on CPU only** — see [Known Issues](#known-issues-and-fixes) |

---

## One-Time Setup

### 1. Install Ollama and pull models

**Install Ollama** (no sudo required — installs to `~/.local`):

```bash
bash ~/ardu_ws/src/drone_nl_control/scripts/install_ollama.sh
```

This downloads Ollama v0.16.3, starts the service, and pulls `gemma3:4b`. It also adds
the required environment variables to `~/.bashrc`.

**Add to your shell environment** (if not already done by the script):

```bash
export PATH="$HOME/.local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib/ollama:${LD_LIBRARY_PATH:-}"
```

**Pull the remaining benchmark models** (required for all three benchmark stages):

```bash
# LLM models
ollama pull qwen2.5:3b
ollama pull llama3.2:3b
ollama pull deepseek-llm:7b

# VLM models
ollama pull llama3.2-vision:11b   # ~8.7 GB — takes a while
ollama pull llava:7b
```

**Verify all models are present:**

```bash
ollama list
```

Expected output (order may vary):

```
NAME                      ID            SIZE    MODIFIED
gemma3:4b                 ...           3.3 GB  ...
qwen2.5:3b                ...           1.9 GB  ...
llama3.2:3b               ...           2.0 GB  ...
deepseek-llm:7b           ...           4.7 GB  ...
llama3.2-vision:11b       ...           8.7 GB  ...
llava:7b                  ...           4.7 GB  ...
```

---

### 2. Build the ROS2 package

> **Important:** Do **not** use `--symlink-install`. It is incompatible with this
> `ament_python` package and causes an "editable install not recognized" error.

```bash
source /opt/ros/humble/setup.bash
cd ~/ardu_ws
colcon build --packages-select drone_nl_control
source install/setup.bash
```

After **any source file change**, re-run both commands above. If `colcon build` fails
with a setup.py-related error, use the manual install fallback:

```bash
cd ~/ardu_ws/src/drone_nl_control
PYTHONPATH=/home/trlspace/ardu_ws/build/drone_nl_control/prefix_override \
  python3 setup.py install \
  --prefix /home/trlspace/ardu_ws/install/drone_nl_control
```

---

## Running a Single Mission

### 1. Start the simulation

Open a dedicated terminal and keep it running throughout the session:

```bash
source ~/ardu_ws/install/setup.bash
ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=false
```

Wait until you see a stable heartbeat message and the Gazebo world has loaded (~30 s).
ArduCopter may crash once on the very first cold start — this is normal. It will restart
automatically; wait for the heartbeat to become stable before proceeding.

### 2. Launch the mission controller

```bash
source ~/ardu_ws/install/setup.bash
ros2 launch drone_nl_control nl_drone.launch.py \
    mission:="find the walking person"
```

Optional flags:

```bash
ros2 launch drone_nl_control nl_drone.launch.py \
    mission:="find the military truck" \
    rviz:=false
```

### 3. Send a mission at runtime (without restarting)

```bash
ros2 topic pub --once /nl_mission std_msgs/String '{"data": "find the red car"}'
```

### 4. Monitor the mission

```bash
# ROS2 status topic
ros2 topic echo /mission_status

# YAML log of every completed mission
ls /tmp/drone_nl_logs/
cat /tmp/drone_nl_logs/mission_*.yaml
```

---

## Running the Full Benchmark

The benchmark has three independent stages plus a final report step. Stages 1 (LLM) and
2 (VLM capture) should be run **before** Stage 3. Stage 3 is the longest by far.

### Step 1: Start the simulation

The simulation must be running and unpaused for Stage 2 (image capture) and Stage 3
(mission trials). Start it in a dedicated terminal and leave it running:

```bash
source ~/ardu_ws/install/setup.bash
ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=false
```

Wait ~30 s for a stable heartbeat before running any benchmark stage.

---

### Step 2: Stage 1 — LLM command validity

Tests whether each LLM reliably generates parseable `Turn(θ)` / `Move(d)` commands from
synthetic drone-state prompts. **Does not require the simulation.**

```bash
source ~/ardu_ws/install/setup.bash
ros2 run drone_nl_control bench_llm
```

This sends 50 randomly generated prompts to each of the four LLM models and measures
what fraction produce valid commands. Results are saved to `~/.drone_benchmark/`.

Expected runtime: ~5–10 minutes (all models are small text-only LLMs).

---

### Step 3: Stage 2 — VLM binary-response validity

Tests whether each VLM reliably answers "Yes" or "No" to detection queries over real
camera frames. Requires the simulation for the **first run only** (to capture test
images); subsequent runs use the saved images offline.

**First run (captures 30 frames from the live camera):**

```bash
source ~/ardu_ws/install/setup.bash
ros2 run drone_nl_control bench_vlm \
    --ros-args -p capture_images:=true
```

**Subsequent runs (uses saved images — no simulation needed):**

```bash
ros2 run drone_nl_control bench_vlm
```

The 30 captured frames are saved to `~/.drone_benchmark/test_images/` and reused
automatically on every future run.

Expected runtime: ~5 minutes for Gemma3, ~15 minutes for Llama3.2-Vision, ~10 minutes
for LLaVA (CPU-only). Total: ~30 minutes.

> **Note on LLaVA:** `llava:7b` must be forced to CPU during image queries — its CLIP
> vision encoder crashes with SIGSEGV on CUDA. This is handled automatically by
> `bench_vlm.py` and `vlm_agent.py` via `num_gpu: 0` in the Ollama request options.

---

### Step 4: Stage 3 — Mission success rate

Runs `mission_trials` full drone missions for every LLM × VLM pair (4 LLMs × 3 VLMs =
12 combinations, 3 trials each = 36 total trials by default).

**The simulation must be running and unpaused.**

Because this stage takes many hours (each `llama3.2-vision:11b` query takes ~54 s, so a
single 15-step trial takes up to 13 minutes), run it with unbuffered output so you can
monitor progress in real time:

```bash
source ~/ardu_ws/install/setup.bash
PYTHONUNBUFFERED=1 ros2 run drone_nl_control bench_mission
```

To override the number of trials from the command line:

```bash
PYTHONUNBUFFERED=1 ros2 run drone_nl_control bench_mission \
    --ros-args -p trials:=3
```

**Estimated runtimes per combo (3 trials, 15 max steps):**

| VLM | Query latency | Max time per trial | Total (3 trials) |
|-----|--------------|-------------------|-----------------|
| Gemma3 | ~18 s | ~5 min | ~15 min |
| Llama3.2-Vision | ~54 s | ~14 min | ~42 min |
| LLaVA (CPU) | ~18 s | ~5 min | ~15 min |

With 4 LLM rows × 3 VLM columns = 12 combos, **total wall time is roughly 8–10 hours**
when all trials exhaust the 15-step limit.

#### Monitoring progress while the benchmark runs

Open a second terminal and run the progress watcher:

```bash
python3 /tmp/watch_bench.py
```

Or poll the output file manually every 30 s:

```bash
# Replace <task-id> with the actual output file path
watch -n 30 "grep -c 'result=' ~/.drone_benchmark/... 2>/dev/null || echo 0"
```

The watcher script also auto-runs `bench_report` and sends a desktop notification when
all 36 trials complete.

---

### Step 5: Generate the report

Once all three stages have produced result YAML files, run:

```bash
source ~/ardu_ws/install/setup.bash
ros2 run drone_nl_control bench_report
```

This reads the most recent `llm_results_*.yaml`, `vlm_results_*.yaml`, and
`mission_results_*.yaml` from `~/.drone_benchmark/` and prints three formatted tables:

- LLM command validity (valid%, avg latency)
- VLM binary-response validity (valid%, avg latency)
- Mission success rate (LLM rows × VLM columns)
- Average steps to success (same grid)

It also writes CSV summaries to `~/.drone_benchmark/`.

To point the report at a specific output directory or file:

```bash
ros2 run drone_nl_control bench_report \
    --ros-args \
    -p output_dir:=~/.drone_benchmark \
    -p mission_file:=~/.drone_benchmark/mission_results_20260225_093000.yaml
```

---

### One-shot convenience script

To run all three stages and the report in sequence (unattended, suitable for overnight
runs):

```bash
source ~/ardu_ws/install/setup.bash
bash ~/ardu_ws/src/drone_nl_control/scripts/run_benchmark.sh
```

Flags:

```bash
# Skip stages you've already completed
bash run_benchmark.sh --skip-llm --skip-vlm

# Override trial count
bash run_benchmark.sh --trials=3
```

> The script requires the simulation to already be running when it reaches Stage 2 or
> Stage 3.

---

## Monitoring Progress

### Live output (recommended)

Always launch Stage 3 with `PYTHONUNBUFFERED=1` so print output is flushed immediately:

```bash
PYTHONUNBUFFERED=1 ros2 run drone_nl_control bench_mission
```

Without this flag, Python buffers stdout in large blocks when not attached to a terminal,
making progress invisible for minutes at a time.

### Reading the benchmark status topic

```bash
ros2 topic echo /benchmark_status
```

Publishes a string such as `Gemma3 × LLaVA1.6 trial 2/3` each time a trial starts.

### Counting completed trials manually

```bash
grep -c 'result=' ~/.drone_benchmark/mission_results_*.yaml 2>/dev/null
```

---

## Configuration Reference

All tunable parameters live in:

```
config/benchmark_config.yaml    # benchmark settings
config/agent_config.yaml        # single-mission agent settings
```

### benchmark_config.yaml

```yaml
llm_models:
  - name: "gemma3:4b"
    label: "Gemma3"
  - name: "qwen2.5:3b"
    label: "Qwen2.5"
  - name: "llama3.2:3b"
    label: "Llama3.2"
  - name: "deepseek-llm:7b"
    label: "DeepSeek"

vlm_models:
  - name: "gemma3:4b"
    label: "Gemma3"
  - name: "llama3.2-vision:11b"
    label: "Llama3.2-Vision"
  - name: "llava:7b"
    label: "LLaVA1.6"

benchmark:
  llm_trials: 50          # prompts per LLM model (Stage 1)
  vlm_trials: 30          # image queries per VLM model (Stage 2)
  mission_trials: 3       # full missions per LLM×VLM combo (Stage 3)
  mission_task: "find the walking person"
  output_dir: "~/.drone_benchmark"

ollama:
  base_url: "http://localhost:11434"
  temperature: 0.2

mission:
  max_steps: 15           # decision loop limit per trial
  hover_altitude_m: 3.0   # takeoff altitude (3 m keeps person in camera FOV)
  turn_duration_sec: 2.0
  move_duration_sec: 2.0
  mavlink_connection: "udpin:127.0.0.1:14550"
```

### Key parameters explained

| Parameter | Default | Why it matters |
|-----------|---------|----------------|
| `hover_altitude_m` | `3.0` | At 8 m the ground target falls outside the camera's ±30° vertical FOV. Use 3 m to keep it visible. |
| `max_steps` | `15` | Limits trial duration. Each step with Llama3.2-Vision takes ~54 s, so 15 steps = ~13 min. |
| `mission_trials` | `3` | Trials per combo. More trials = better statistics but much longer runtime. |
| `temperature` | `0.2` | Low temperature → more deterministic command generation. |

---

## Package Structure

```
drone_nl_control/
├── drone_nl_control/
│   ├── drone_interface.py      # MAVLink: connect / arm / takeoff / Turn / Move / RTL
│   ├── vlm_agent.py            # ROS2 node: camera subscriber + Ollama VLM query
│   ├── llm_agent.py            # Stateless: generate and parse LLM commands
│   ├── mission_controller.py   # ROS2 node: orchestrates the decision loop
│   ├── bench_llm.py            # Stage 1: LLM command-validity benchmark
│   ├── bench_vlm.py            # Stage 2: VLM binary-response-validity benchmark
│   ├── bench_mission.py        # Stage 3: End-to-end mission success benchmark
│   └── report.py               # bench_report: reads YAML results, prints tables + CSV
├── launch/
│   └── nl_drone.launch.py      # Starts iris_tracking sim + mission_controller
├── config/
│   ├── benchmark_config.yaml   # All benchmark parameters (models, trials, mission)
│   └── agent_config.yaml       # Single-mission agent parameters
├── scripts/
│   ├── install_ollama.sh       # One-time Ollama + gemma3:4b install (no sudo)
│   └── run_benchmark.sh        # Run all three stages + report in sequence
└── package.xml
```

---

## Known Issues and Fixes

These issues were encountered and resolved during development. They are fixed in the
current codebase but documented here for reference.

### 1. `llava:7b` crashes with SIGSEGV on NVIDIA GPUs

**Symptom:** Every `llava:7b` image query returns HTTP 500. The Ollama log shows
`signal arrived during cgo execution` — a SIGSEGV inside the CLIP vision encoder's CUDA
code.

**Root cause:** The CLIP vision encoder in `llava:7b` is incompatible with Turing-class
GPUs (NVIDIA T1000, compute capability 7.5) when running in GPU mode.

**Fix (already applied):** Both `bench_vlm.py` and `vlm_agent.py` detect the `llava`
model name and add `num_gpu: 0` to the Ollama API request options, forcing CPU inference.
CPU inference is ~2× slower but reliable.

---

### 2. `llama3.2-vision:11b` times out during mission trials

**Symptom:** VLM queries hang for >30 s and the benchmark logs a timeout error.

**Root cause:** The 11 GB model runs split across GPU (~6.8 GB) and CPU (~3.5 GB) on an
8 GB card. CPU offload means inference takes 40–60 s per query instead of the typical
2–5 s.

**Fix (already applied):** `vlm_agent.py` uses a 120 s request timeout (previously 30 s).

**Practical note:** Each 15-step trial takes up to 13 minutes with this VLM. Plan for
~7 hours of wall time for the 12-combo × 3-trial benchmark if most trials reach the step
limit.

---

### 3. Drone drifts between trials (position not reset)

**Symptom:** After a trial ends, the drone is displaced from its starting position (due
to `Move()` commands). Subsequent trials start from the wrong location.

**Fix (already applied):** `bench_mission.py` calls `drone.return_to_launch()` between
trials. This sends an RTL MAVLink command and waits for the drone to land and disarm
before starting the next trial.

---

### 4. NAV_TAKEOFF fails on cold start

**Symptom:** `NAV_TAKEOFF: FAILED` on the first run after launching the simulation.

**Root cause:** NAV_TAKEOFF requires the EKF to be in GPS navigation mode. On a cold
start the EKF may not have acquired GPS lock by the time arming completes.

**Fix (already applied):** `drone_interface.py` waits for 30 s of sustained GPS fix
before arming, retries arming up to 10 times, and retries NAV_TAKEOFF up to 3 times with
re-arming between attempts.

**Workaround if it still fails:** Let the simulation run for ~60 s before starting the
benchmark, or restart `bench_mission` — the SITL state persists between runs.

---

### 5. ArduCopter crashes once on first cold boot

**Symptom:** MAVProxy shows `[Errno 111] Connection refused` immediately after the first
launch. ArduCopter exits with code 1.

**Root cause:** ArduCopter starts before Gazebo's JSON physics interface (port 9002) is
ready, crashes, and then restarts. On the second start Gazebo is ready and everything
proceeds normally.

**Workaround:** Simply wait ~30–60 s after `ros2 launch ardupilot_gz_bringup
iris_tracking.launch.py`. The process restarts automatically.

---

### 6. `colcon build --symlink-install` fails

**Symptom:** `error: option --editable not recognized` or `--uninstall not recognized`
during build.

**Root cause:** The `setup.py` in this package does not support editable installs.

**Fix:** Never use `--symlink-install`:

```bash
colcon build --packages-select drone_nl_control   # correct
colcon build --packages-select drone_nl_control --symlink-install   # WRONG
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `[MAV] EKF/GPS timeout — proceeding anyway` | MAVProxy reconnecting during EKF wait | Wait ~60 s for simulation to fully stabilise |
| `NAV_TAKEOFF: FAILED` persists after retries | EKF not in GPS mode | Restart bench_mission; SITL state is preserved |
| VLM always returns "no" for all steps | Person below camera FOV | Set `hover_altitude_m: 3.0` in benchmark_config.yaml (not 8.0) |
| `llava` queries all return HTTP 500 | CLIP vision encoder SIGSEGV on GPU | Ensure `num_gpu: 0` is set for llava in vlm_agent.py and bench_vlm.py |
| `llama3.2-vision` queries timeout | Model too large for GPU alone | Ensure VLM timeout is 120 s (`vlm_agent.py` line ~98) |
| Benchmark output appears frozen / no progress | Python stdout buffering | Always prefix with `PYTHONUNBUFFERED=1` |
| `LLM parse failed` repeatedly | Model generating prose instead of commands | Check `ollama ps` — model may be unloaded; verify `ollama serve` is running |
| `PackageNotFoundError: drone_nl_control` | Install metadata missing | Re-run `colcon build --packages-select drone_nl_control && source install/setup.bash` |
| `[Errno 111] Connection refused` in MAVProxy | ArduCopter crashed before Gazebo ready | Normal on cold start; wait ~60 s for automatic restart |

---

## Citation

This package implements the algorithm described in:

> **Taking Flight with Dialogue: Enabling Natural Language Control for Drones**
> Nilufar Umarov, Jan Koch, Taras Bondar, Igor Polyakov
> arXiv:2404.04781, 2024
