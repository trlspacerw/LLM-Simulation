# ngrok + MAVLink Drone Platform — Setup & Run Guide

Complete record of what was built, installed, tested, and how to run it.

- **Project location:** `~/ardu_ws/src/drone_development/drone-platform/`
- **Machine:** Ubuntu 22.04, Python 3.10, ROS 2 Humble (unused here — the
  platform talks pymavlink directly), Gazebo Harmonic 8.10, ArduPilot SITL
  at `~/ardupilot`
- **Status:** smoke test **7/7 PASS** (2026-06-12), re-verified through the
  one-command `start_all.sh` flow and the bring-your-own-sim
  `drone_client.py` flow. ngrok authtoken **not yet configured** — the only
  remaining step for remote access (see section 5).

---

## 1. What this is

A remote drone control platform: control and monitor an ArduPilot SITL
drone (simulated in Gazebo) from anywhere on the internet, through two
independent paths:

1. **Raw MAVLink path** — ngrok TCP tunnel → mavlink-router TCP endpoint,
   so QGroundControl / Mission Planner on a remote machine connect directly.
2. **Web gateway path** — FastAPI server speaking MAVLink via pymavlink,
   exposing a REST API + WebSocket telemetry stream through an ngrok HTTPS
   tunnel, consumed by a single-file browser dashboard.

```
Gazebo + ArduPilot SITL
        │ (MAVLink UDP 14550)
   mavlink-router
   ├── UDP 14551  → FastAPI gateway (pymavlink)
   ├── TCP 5761   → raw MAVLink endpoint   (5760 is taken by SITL itself!)
   └── UDP 14552  → local GCS (optional, for debugging)
        │
   ngrok agent
   ├── tcp 5761   → remote QGroundControl / Mission Planner
   └── http 8000  → remote web dashboard (REST + WebSocket)
```

> **Latency / scope.** ngrok adds ~100–500 ms round trip. This platform is
> for **monitoring and high-level commands** (guided waypoints, takeoff,
> RTL) — **never** manual RC-style joystick control over the tunnel.

The MAVLink fan-out in the middle can be either **mavlink-router** (used by
`start_all.sh` and the manual scripts) or **your own MAVProxy** with
`--out=127.0.0.1:14551 --out=tcpin:0.0.0.0:5761` (the
bring-your-own-simulation mode, section 5) — the gateway and tunnels don't
care which one feeds them.

## 2. Project layout

```
drone-platform/
├── drone_client.py        # bring-your-own-sim mode: gateway + ngrok in one command,
│                          #   for use with your ROS 2 launch + your own MAVProxy
├── server/
│   ├── main.py            # FastAPI app: serves the dashboard at /, /api/state,
│   │                      #   /ws/telemetry, /healthz
│   ├── mavlink_client.py  # pymavlink manager: single RX thread, locked sends,
│   │                      #   1 Hz GCS heartbeat, 5 s heartbeat watchdog + auto-reconnect
│   ├── telemetry.py       # latest-state store + 2 Hz WebSocket broadcast,
│   │                      #   STATUSTEXT & link up/down forwarded immediately
│   ├── commands.py        # arm/disarm/mode/takeoff/goto/rtl/land, COMMAND_ACK handling
│   └── auth.py            # X-API-Key middleware (?api_key= on the WS) + CORS
├── client/
│   └── index.html         # single-file dashboard: Leaflet map, breadcrumb trail,
│                          #   telemetry panel, command buttons, click-map-to-goto
├── config/
│   ├── mavlink-router.conf
│   └── ngrok.yml          # v3 endpoints: tcp→5761, https→8000
├── scripts/
│   ├── start_all.sh       # ONE-COMMAND startup (headless Gazebo→SITL→router→gateway),
│   │                      #   flags: --gui, --ngrok, <world.sdf>
│   ├── stop_all.sh        # stop everything (escalates to SIGKILL for arducopter)
│   ├── start_sitl.sh      # manual mode: sim_vehicle.py, Gazebo JSON backend
│   ├── start_router.sh    # manual mode
│   ├── start_server.sh    # manual mode
│   ├── start_ngrok.sh     # manual mode
│   └── smoke_test.py      # end-to-end flight test (stdlib only)
├── logs/                  # runtime logs from start_all.sh / drone_client.py (git-ignored)
├── requirements.txt       # fastapi, uvicorn[standard], pymavlink, python-dotenv
├── .env                   # API_KEY etc. — created automatically, git-ignored, NEVER commit
├── .env.example
└── README.md              # full API reference & docs
```

## 3. What was installed (no sudo — all user-local)

| Tool | Version | How | Where |
|------|---------|-----|-------|
| meson + ninja | 1.11 / 1.13 | `pip install --user` | `~/.local/bin` |
| **mavlink-routerd** | master `2362c62` | built from [mavlink-router](https://github.com/mavlink-router/mavlink-router) source, `meson setup build --prefix=$HOME/.local` | `~/.local/bin/mavlink-routerd` |
| **ngrok** | 3.39.7 | official v3 tarball | `~/.local/bin/ngrok` |

Python deps (fastapi 0.135, pydantic 2.12, pymavlink, uvicorn, dotenv) were
already present system-wide. `~/.local/bin` is already on PATH.

A `.env` was generated with a random `API_KEY` (`openssl rand -hex 24`).
Read it with: `grep ^API_KEY= drone-platform/.env`

## 4. Fixes discovered during testing

- **Port 5760 conflict (real bug, fixed):** SITL's own `serial0` listens on
  TCP 5760 on the same machine, so mavlink-router could not bind it. The
  router's TCP server moved to **5761** (`mavlink-router.conf`,
  `ngrok.yml`, README all updated). Remote users are unaffected — they only
  see the ngrok-assigned address.
- **`--no-extra-ports` added to `start_sitl.sh`:** without it, MAVProxy
  adds default outputs on 14550 *and 14551*, bypassing the router and
  colliding with the gateway's listen port.
- **Headless note:** for unattended runs, replace `--console` with
  `--mavproxy-args="--daemon"` in the sim_vehicle command (the smoke test
  was run this way, with `gz sim -s -r iris_runway.sdf` as the headless
  Gazebo server). `start_all.sh` does this automatically.
- **`arducopter` ignores SIGTERM:** plain kill leaves it holding TCP 5760.
  `stop_all.sh` waits a few seconds, then escalates to SIGKILL.
- **Dashboard is served by the gateway:** `GET /` returns
  `client/index.html`, so one URL (local or ngrok) gives UI + API, and the
  page pre-fills its own base URL. No file copying or CORS friction needed.

## 5. How to run

### The easy way — one command

```bash
cd ~/ardu_ws/src/drone_development/drone-platform
scripts/start_all.sh             # headless; add --gui for Gazebo's window,
                                 # --ngrok to also start the tunnels
```

It starts Gazebo → SITL → router → gateway in the background (logs in
`logs/`), waits until the MAVLink link is up, and prints the dashboard URL
and API key. Then open **http://127.0.0.1:8000** — the gateway serves the
dashboard itself with the URL pre-filled; paste the key and Connect.
Stop everything with `scripts/stop_all.sh`.

### Bring your own simulation — `drone_client.py`

When you launch the sim yourself (ROS 2 bringup + RViz) and want MAVProxy's
console/map locally, skip `start_all.sh` and mavlink-router entirely:

```bash
# terminal 1 — your simulation
cd ~/ardu_ws && source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=true

# terminal 2 — your MAVProxy (does the routing)
mavproxy.py --master=udp:127.0.0.1:14550 \
            --out=127.0.0.1:14551 \
            --out=tcpin:0.0.0.0:5761 \
            --console --map

# terminal 3 — web gateway + ngrok
python3 drone_client.py            # --no-ngrok for local-only
```

The `14551` out feeds the web gateway; `tcpin:0.0.0.0:5761` serves the
ngrok TCP tunnel for remote QGroundControl. `drone_client.py` prints the
dashboard URL, API key and tunnel addresses, and reports vehicle link
up/down. Ctrl+C stops the web side only — the sim keeps running.

### The manual way — step by step (for debugging)

Each step in its own terminal, in this order
(`cd ~/ardu_ws/src/drone_development/drone-platform` first):

### Step 1 — Gazebo world

Your usual workflow, e.g. headless:

```bash
gz sim -s -r iris_runway.sdf        # plugin/resource paths already in your env
```

(or any world containing a model with the ArduPilot plugin)

### Step 2 — SITL

```bash
scripts/start_sitl.sh
```

Runs `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
--no-extra-ports --out=udp:127.0.0.1:14550`. Wait for "EKF3 IMU0 is using GPS".

### Step 3 — MAVLink router

```bash
scripts/start_router.sh
```

Expect: UDP server 14550, UDP clients 14551/14552, **TCP server 5761**.

### Step 4 — FastAPI gateway

```bash
scripts/start_server.sh
```

Expect `MAVLink link UP` in the log within seconds. Quick checks:

```bash
curl -s http://127.0.0.1:8000/healthz          # {"ok":true,"mavlink_connected":true}
KEY=$(grep ^API_KEY= .env | cut -d= -f2)
curl -s -H "X-API-Key: $KEY" http://127.0.0.1:8000/api/state | python3 -m json.tool
```

### Step 5 — ngrok tunnels (remote access)

**One-time setup (still pending on this machine):**

1. Get your token at <https://dashboard.ngrok.com/get-started/your-authtoken>
2. `ngrok config add-authtoken <YOUR_TOKEN>`
3. **Verify the free account** in the ngrok dashboard — required for the
   TCP tunnel (the HTTPS one works unverified).

Then:

```bash
scripts/start_ngrok.sh
```

Read the assigned public addresses from the ngrok terminal UI, or:

```bash
curl -s http://127.0.0.1:4040/api/tunnels | python3 -m json.tool   # "public_url" fields
```

## 6. Connecting remotely

**QGroundControl (raw MAVLink):** Application Settings → Comm Links → Add →
Type **TCP** → Host = your assigned `N.tcp.ngrok.io` (no `tcp://` prefix),
Port = the assigned port → Connect. Mission Planner: same host/port via the
TCP option in the connection dropdown.

**Browser dashboard:** served by the gateway itself — open
**http://127.0.0.1:8000** locally or your **https://….ngrok-free.app** URL
remotely. The base-URL field is pre-filled; paste the API key and hit
**Connect**. (`client/index.html` is also fully static, so copying the
file to another machine and opening it directly works too.) The dashboard
sends `ngrok-skip-browser-warning` so the free-plan interstitial doesn't
break API calls.

**API quick reference** (full table in `README.md`): all `/api/*` need the
`X-API-Key` header; the WebSocket needs `?api_key=`. `POST /api/arm`,
`/api/disarm`, `/api/mode {"mode":"GUIDED"}`, `/api/takeoff {"altitude":10}`
(refused unless GUIDED + armed + 3D fix), `/api/goto {"lat","lon","alt"}`,
`/api/rtl`, `/api/land`; `GET /api/state`; `WS /ws/telemetry` (state at
2 Hz + immediate status/link events). Errors: 401 bad key, 409 refused or
autopilot-rejected, 503 link down, 504 no COMMAND_ACK.

## 7. Smoke test (verified working)

With steps 1–4 up (ngrok not needed):

```bash
python3 scripts/smoke_test.py
```

Result on 2026-06-12 against the real Gazebo + SITL stack:

```
PASS  link up           (mode=STABILIZE)
PASS  3D GPS fix        (fix=6 sats=10)
PASS  mode GUIDED
PASS  arm               (armed on attempt 1)
PASS  takeoff 10 m      (alt_rel=9.2 m)
PASS  goto 30 m north   (reached target, 3.3 m off)
PASS  RTL               (landed + auto-disarmed)
```

Also verified: raw MAVLink heartbeats over TCP 5761 via pymavlink, 401 on
missing API key, 503 when SITL is down, and the gateway auto-reconnecting
through a router restart (link-down/up events pushed to the WebSocket).

## 8. Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| Router: `Could not bind to tcp socket` | Something on 5761 (or you reverted to 5760 — SITL owns that). `ss -tlnp \| grep 576` |
| Gateway stuck `waiting for MAVLink heartbeat` | Router not running, or SITL launched without `--out=udp:127.0.0.1:14550` |
| `/api/takeoff` → 409 | By design: needs mode GUIDED + armed + 3D GPS fix (wait for EKF) |
| Arm rejected right after boot | Pre-arm checks settling — retry after a few seconds (smoke test retries automatically) |
| ngrok `mavlink` endpoint fails to start | Account not verified (TCP tunnels need a verified free account) |
| `drone_client.py`: "tunnels NOT UP — authtoken set?" | Run `ngrok config add-authtoken <token>` once, then restart it |
| `drone_client.py`: "vehicle link: DOWN" forever | Your MAVProxy isn't running or lacks `--out=127.0.0.1:14551` |
| `start_all.sh`: "ports already in use" | A previous run is still up — `scripts/stop_all.sh` first |
| ngrok config version mismatch when merging configs | `ngrok config upgrade` |
| Dashboard gets HTML instead of JSON via ngrok | Free-plan interstitial; the dashboard already sends `ngrok-skip-browser-warning` |
| SITL restarted, dashboard shows "vehicle link: DOWN" | Expected — gateway reconnects automatically within ~5 s of the new heartbeat |

## 9. Security notes

- API key lives only in `.env` (git-ignored); rotate by editing `API_KEY`
  and restarting the server.
- Anyone with the ngrok **TCP** address speaks raw, unauthenticated MAVLink
  to the vehicle — treat the address as a secret, stop the tunnel when idle.
- Once you know your ngrok HTTPS URL, set `ALLOWED_ORIGINS` in `.env` to it
  instead of `*`.
