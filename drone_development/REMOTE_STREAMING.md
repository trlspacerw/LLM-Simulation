# Remote Camera + MAVLink Streaming (no ROS)

End-to-end recipe for piping the Gazebo (Harmonic) camera and ArduPilot SITL
MAVLink to a remote machine over **ngrok TCP tunnels**, using a single Python
process on the sim host.

```
   sim host                                                    remote host
 ┌──────────────────────────────────┐                        ┌───────────────┐
 │ gz sim   ──gz topic──►           │                        │               │
 │ (iris_tracking world)            │                        │ viewer.py     │
 │                                  │   ngrok tcp 8554       │   (JPEG)      │
 │ gazebo_streamer.py ──tcp:8554────┼──────────────────────►│               │
 │      │                           │                        │ pymavlink     │
 │      └─────────tcp:5772──────────┼──── ngrok tcp 5772 ───►│   client      │
 │                                  │                        │               │
 │ arducopter (SITL) ─tcp:5763──────┘                        └───────────────┘
 │   (also tcp:5760 used by MAVProxy)
 └──────────────────────────────────┘
```

The Python streamer subscribes to the Gazebo camera via `gz-transport13`
Python bindings (no ROS topic, no v4l2 device, no GStreamer) and exposes it
as a length-prefixed JPEG stream. The MAVLink half is a transparent TCP
byte-relay to one of SITL's spare TCP ports (5763 by default), so any
`pymavlink` code works unchanged — only the connection URI changes.

---

## Files

**Single-drone, remote streaming over ngrok:**

| File | Role | Runs on |
|---|---|---|
| `gazebo_streamer.py` | Camera + MAVLink server, single process | **sim host** |
| `remote_camera_viewer.py` | JPEG stream receiver / recorder | remote host |
| `remote_demo_client.py` | Minimal example: heartbeat + camera frame | remote host |
| `gazebo_camera_stream.py` | Camera-only variant (kept for reference) | sim host |

**Single-drone, local (same machine, no ngrok):**

| File | Role |
|---|---|
| `connect_vehicle_with_camera.py` | pymavlink + gz-transport in one process, shows live feed |

**Two independent drones, local:**

| File | Role |
|---|---|
| `launch_two_drones.sh` | Starts gz sim + two ArduCopter SITL instances (no ROS) |
| `connect_two_drones.py` | Side-by-side camera windows + MAVLink for both drones |
| `two_drones_data.py` | Minimal: MAVLink attitude stream from both drones |
| `two_drones_camera.py` | Minimal: both Gazebo camera feeds in cv2 windows |

---

## Prerequisites (sim host)

Already present on this machine, but for reference:

```bash
# Gazebo Harmonic Python bindings
sudo apt install python3-gz-transport13 python3-gz-msgs10

# Python deps
pip install opencv-python pymavlink numpy
```

ngrok account + auth token (free tier is enough for 640×480 @ 15 fps):

```bash
ngrok config add-authtoken <YOUR_TOKEN>
```

---

## Step 1 — Start the simulation

From `~/ardu_ws`:

```bash
source install/setup.bash
ros2 launch drone_nl_control nl_drone.launch.py
```

Wait for `gz sim`, `arducopter`, and MAVProxy to come up (about 5-10 s).
Sanity-check the camera topic is live:

```bash
gz topic -l | grep front_camera/image
# /world/iris_tracking/model/iris/link/front_camera_link/sensor/front_camera/image
```

> The launch uses ROS only as a process orchestrator — runtime data flow
> (camera + MAVLink) does **not** go through ROS.

---

## Step 2 — Start the unified streamer

In a second terminal:

```bash
cd ~/ardu_ws/src/drone_development
python3 gazebo_streamer.py
```

Expected output:

```
[cam ] subscribing to: /world/iris_tracking/.../front_camera/image
[cam ] listening on 0.0.0.0:8554  (ngrok tcp 8554)
[mav ] listening on 0.0.0.0:5772  (relay -> 127.0.0.1:5763) (ngrok tcp 5772)
```

Useful flags:

| Flag | Default | Purpose |
|---|---|---|
| `--cam-port` | 8554 | local TCP port for camera |
| `--mav-port` | 5772 | local TCP port for MAVLink (don't pick 5760-5763, SITL owns those) |
| `--sitl-port` | 5763 | which SITL TCP port the relay attaches to (5760 is taken by MAVProxy) |
| `--fps` | 15 | camera frame cap |
| `--quality` | 70 | JPEG quality 1-100 |
| `--no-camera` | off | run MAVLink relay only |
| `--no-mavlink` | off | run camera only |

---

## Step 3 — Local sanity test (recommended before ngrok)

```bash
# camera
python3 remote_camera_viewer.py --host 127.0.0.1 --port 8554
# expect window opening, or use --no-show for headless throughput print

# mavlink
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('tcp:127.0.0.1:5772')
print(m.wait_heartbeat(timeout=10))
"
```

Confirmed working numbers from this machine:
- Camera: **15 fps**, **~13.5 KB/frame** (~1.6 Mbps)
- MAVLink: HEARTBEAT received in <1 s

---

## Step 4 — Expose via ngrok

Two tunnels, one per port:

```bash
# terminal A
ngrok tcp 8554
# Forwarding: tcp://0.tcp.ngrok.io:NNNNN  ->  localhost:8554

# terminal B
ngrok tcp 5772
# Forwarding: tcp://0.tcp.ngrok.io:MMMMM  ->  localhost:5772
```

Note the two `host:port` pairs ngrok gives you.

> ngrok's free tier allows multiple tunnels but limits concurrent connections.
> One viewer + one MAVLink client per tunnel is fine.

---

## Step 5 — Connect from the remote machine

Install on the remote:
```bash
pip install opencv-python pymavlink numpy
```

Copy `remote_camera_viewer.py` and `remote_demo_client.py` to the remote.

```bash
# camera (display)
python3 remote_camera_viewer.py --host 0.tcp.ngrok.io --port NNNNN

# camera (headless + save to MP4)
python3 remote_camera_viewer.py --host 0.tcp.ngrok.io --port NNNNN \
    --no-show --save flight.mp4

# combined demo: heartbeat + arm + 1 frame snapshot
python3 remote_demo_client.py \
    --mav-host 0.tcp.ngrok.io --mav-port MMMMM \
    --cam-host 0.tcp.ngrok.io --cam-port NNNNN
```

Your existing scripts (`takeoff-land.py`, `set-yaw.py`, etc.) work unchanged
on the remote — just replace any `tcp:127.0.0.1:5760` URI with
`tcp:<ngrok-host>:<port>`.

---

## Wire format (camera)

Length-prefixed framing over plain TCP:

```
┌────────────┬────────────────────┐
│ uint32 BE  │  JPEG bytes (N)    │  ... repeat
│  = N       │                    │
└────────────┴────────────────────┘
```

Trivial to consume in any language. See `remote_camera_viewer.py` for the
reference reader (≤10 lines).

---

## Troubleshooting

**`Address already in use` on 5772**
Old streamer still running. `pkill -f gazebo_streamer.py` and restart.

**MAVLink relay connects but immediately drops with `EOF on TCP socket`**
SITL only allows one client per TCP port. Pick a port nobody else owns:
- `5760` → owned by MAVProxy
- `5762`, `5763` → spare (default `--sitl-port` is 5763)

Check with:
```bash
ss -tnp | grep -E ":576[0-3]"
```

**`No image topics found`**
The sim isn't up yet, or it crashed. Verify with `gz topic -l | grep image`.

**Frames stutter through ngrok**
Lower bandwidth: `--fps 10 --quality 50`.

**Want to keep using your local UDP 14550 client too**
Already works. The relay attaches to 5763, MAVProxy keeps fanning 5760 → UDP
14550/14551 untouched.

---

---

# Multi-vehicle Setup (Two Independent Drones)

Spawns two `iris_with_front_cam` instances in one Gazebo world, each driven
by its own ArduCopter SITL process. No ROS — `launch_two_drones.sh` runs
`gz sim` and two `arducopter` processes directly. Each drone has a distinct
SYSID, distinct MAVLink ports, distinct Gazebo camera topic, and distinct
FDM ports, so they cannot interfere with each other.

```
       Gazebo (iris_tracking_two world)
       ┌──────────────────────────────────────┐
       │  model "iris"        model "iris2"   │
       │  pose (0,0)          pose (5,0)      │
       │      │                   │           │
       │      └─ camera ─►        └─ camera ─►│   gz-transport image topics
       └──────────┬───────────────────┬───────┘
                  │                   │
              FDM 9002            FDM 9012
                  │                   │
        arducopter --instance 0   arducopter --instance 1
        --sysid 1                 --sysid 2
        MAVLink TCP 5760          MAVLink TCP 5770
```

## Per-drone resource map

|                     | Drone 1 (iris)            | Drone 2 (iris2)            |
|---|---|---|
| Gazebo model name   | `iris`                    | `iris2`                    |
| Spawn pose          | `(0, 0, 0.195)`           | `(5, 0, 0.195)`            |
| MAVLink TCP (primary) | `tcp:127.0.0.1:5760`    | `tcp:127.0.0.1:5770`       |
| MAVLink TCP (extra) | 5762, 5763                | 5772, 5773                 |
| FDM port            | 9002                      | 9012                       |
| SYSID               | 1                         | 2                          |
| Camera topic        | `/world/iris_tracking_two/model/iris/link/front_camera_link/sensor/front_camera/image`  | `/world/iris_tracking_two/model/iris2/link/front_camera_link/sensor/front_camera/image` |

## New / modified assets

Created (nothing existing is touched):

| Path | What it is |
|---|---|
| `src/ardupilot_gazebo/models/iris_with_gimbal_inst1/` | Copy of `iris_with_gimbal/` with `fdm_port_in` patched to **9012** |
| `src/ardupilot_gazebo/models/iris2_with_front_cam/` | Wrapper model that includes the `_inst1` gimbal |
| `src/ardupilot_gazebo/worlds/iris_tracking_two.sdf` | World file with both drones spawned 5 m apart |

After adding these, rebuild so they end up under `install/`:

```bash
cd ~/ardu_ws
colcon build --packages-select ardupilot_gazebo
```

## Step 1 — Launch the two-drone sim

```bash
bash ~/ardu_ws/src/drone_development/launch_two_drones.sh
```

Expected:

```
[launcher] starting gz sim with iris_tracking_two.sdf
[launcher] gz sim ready
[launcher] starting arducopter instance 0 (drone 1, SYSID 1)
[launcher] starting arducopter instance 1 (drone 2, SYSID 2)
[launcher] all up. logs in /tmp/two_drones/
```

Ctrl+C in this terminal triggers a `cleanup` trap that kills both arducopter
processes and `gz sim`.

## Step 2 — Minimal Python (telemetry only)

`two_drones_data.py` — one thread per drone, both connect over plain TCP,
both request data streams at 4 Hz, both print attitude in real time:

```python
import threading
from pymavlink import mavutil

def stream(uri, tag):
    m = mavutil.mavlink_connection(uri)
    m.wait_heartbeat()
    print(f"[{tag}] connected sys={m.target_system}")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    while True:
        msg = m.recv_match(type='ATTITUDE', blocking=True)
        print(f"[{tag}] roll={msg.roll:+.2f} pitch={msg.pitch:+.2f} yaw={msg.yaw:+.2f}")

for uri, tag in [("tcp:127.0.0.1:5760", "drone1"),
                 ("tcp:127.0.0.1:5770", "drone2")]:
    threading.Thread(target=stream, args=(uri, tag), daemon=True).start()
threading.Event().wait()
```

Run:
```bash
python3 ~/ardu_ws/src/drone_development/two_drones_data.py
```

Expected output (interleaved across threads):
```
[drone1] connected sys=1
[drone2] connected sys=2
[drone1] roll=-0.00 pitch=-0.00 yaw=-0.01
[drone2] roll=-0.00 pitch=-0.00 yaw=-0.00
...
```

## Step 3 — Minimal Python (camera only)

`two_drones_camera.py` — subscribes to both Gazebo camera topics, displays
each in its own OpenCV window:

```python
import cv2, numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

frames = {}
def make_cb(name):
    def cb(msg):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        frames[name] = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return cb

W = "iris_tracking_two"
node = Node()
node.subscribe(Image, f"/world/{W}/model/iris/link/front_camera_link/sensor/front_camera/image",  make_cb("drone1"))
node.subscribe(Image, f"/world/{W}/model/iris2/link/front_camera_link/sensor/front_camera/image", make_cb("drone2"))

while True:
    for name, f in list(frames.items()):   # list() avoids race with bg thread
        cv2.imshow(name, f)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

Run:
```bash
python3 ~/ardu_ws/src/drone_development/two_drones_camera.py
```

Two windows open; press **q** in either to quit.

## Step 4 — Combined demo

`connect_two_drones.py` wraps each drone in a `Drone` class (MAVLink +
camera in one object) and runs both side-by-side with live telemetry
printing. Useful template for higher-level mission code.

```bash
python3 ~/ardu_ws/src/drone_development/connect_two_drones.py
```

## Adding a third drone

Same pattern, repeat for instance 2:

1. Copy `iris_with_gimbal_inst1/` → `iris_with_gimbal_inst2/`, set
   `fdm_port_in` to **9022**.
2. Copy `iris2_with_front_cam/` → `iris3_with_front_cam/`, point its
   `<include>` at `iris_with_gimbal_inst2`.
3. Add a third `<include>` in `iris_tracking_two.sdf` (or fork to a new
   world) with `<name>iris3</name>` at a non-overlapping pose.
4. In `launch_two_drones.sh` add a third `arducopter --instance 2
   --sysid 3 ...` block. MAVLink will land on **tcp:5780**.

The pattern: each new drone instance N needs `fdm_port_in = 9002 + 10*N`,
unique model dir name, unique gz model `<name>`, `--instance N` and
`--sysid N+1` on arducopter.

## Multi-drone troubleshooting

**Drone 2's MAVLink connects but heartbeat shows sys=1**
`--sysid 2` flag missing on the second `arducopter`. The .parm file
workaround does not work because `SYSID_THISMAV` is only loaded after
boot from EEPROM, and the param-file load happens too late for the
initial heartbeats.

**`Closed connection on SERIAL0` in `sitl0.log` / `sitl1.log`**
An older `arducopter` process is still bound to 5760/5770. Hard-kill
leftovers:
```bash
pkill -9 -f "gz sim|arducopter|launch_two_drones"
```

**Stale `/world/iris_tracking/...` topic shows up in `gz topic -l`**
Cached advertisement from a prior single-drone run. The real topics for
the two-drone world are `/world/iris_tracking_two/...` and they work
fine even when the stale entry is listed.

**Both drones drift apart on takeoff**
They share the same GPS home location but each has its own EKF. To pin
them, edit `iris_tracking_two.sdf` `<spherical_coordinates>` or set
`GPS_AUTO_CONFIG=0` in the parm files.

---

## Shutdown

```bash
pkill -f gazebo_streamer.py
pkill -f "ros2 launch|gz sim|arducopter|mavproxy|rviz2|launch_two_drones"
```
