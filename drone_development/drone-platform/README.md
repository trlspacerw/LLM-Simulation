# Drone Platform — remote control & monitoring over ngrok

Control and monitor an ArduPilot SITL drone (Gazebo Harmonic) from anywhere
on the internet, via two paths:

1. **Raw MAVLink** — an ngrok TCP tunnel to a MAVLink TCP endpoint, so
   QGroundControl / Mission Planner on a remote machine connect directly.
2. **Web gateway** — a FastAPI server speaking MAVLink via pymavlink,
   exposing a REST API + WebSocket telemetry stream over an ngrok HTTP
   tunnel, consumed by the single-file browser dashboard in `client/`.

```
Gazebo + ArduPilot SITL
        │ (MAVLink UDP 14550)
   mavlink-router
   ├── UDP 14551  → FastAPI gateway (pymavlink)
   ├── TCP 5761   → raw MAVLink endpoint
   └── UDP 14552  → local GCS (optional, for debugging)
        │
   ngrok agent
   ├── tcp 5761   → remote QGroundControl
   └── http 8000  → remote web dashboard (REST + WebSocket)
```

> **Latency / scope.** ngrok adds roughly 100–500 ms round trip. This
> platform is for **monitoring and high-level commands** (guided-mode
> waypoints, takeoff, RTL). It is **NOT** suitable for manual RC-style
> joystick control — never attempt to fly it that way over the tunnel.

## Prerequisites

- Ubuntu 22.04, Python 3.10
- ArduPilot SITL (`sim_vehicle.py` on PATH) + Gazebo Harmonic with the
  `ardupilot_gazebo` plugin (already installed in your stack)
- [mavlink-router](https://github.com/mavlink-router/mavlink-router)
  (`mavlink-routerd` on PATH)
- [ngrok v3 agent](https://ngrok.com/download) and a free ngrok account.
  **TCP tunnels require a verified free account** (verify in the ngrok
  dashboard, then `ngrok config add-authtoken <YOUR_TOKEN>` once).

## Install

```bash
cd drone-platform
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

cp .env.example .env
# set API_KEY in .env, e.g.:
sed -i "s/^API_KEY=.*/API_KEY=$(openssl rand -hex 24)/" .env
```

Never commit `.env`; the API key lives only there (and in your remote
clients).

## Quick start (one command)

```bash
scripts/start_all.sh             # headless Gazebo + SITL + router + gateway
scripts/start_all.sh --gui       # same, with the Gazebo GUI
scripts/start_all.sh --ngrok     # also start the tunnels and print public URLs
scripts/stop_all.sh              # stop everything
```

On first run it creates `.env` with a fresh API key. It health-checks the
whole chain, then prints the dashboard URL and the API key. Open
**http://127.0.0.1:8000** — the gateway serves the dashboard itself, the
URL field is pre-filled, just paste the key and hit Connect. Logs are in
`logs/`. Give the EKF ~20–30 s after startup before arming.

## Bring your own simulation (`drone_client.py`)

If you start Gazebo/SITL yourself (e.g. a ROS 2 bringup with RViz) and run
MAVProxy with its console and map, you don't need `start_all.sh` or
mavlink-router at all — MAVProxy does the routing:

```bash
# terminal 1 — your simulation
cd ~/ardu_ws && source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_tracking.launch.py rviz:=true

# terminal 2 — your MAVProxy, feeding the platform
mavproxy.py --master=udp:127.0.0.1:14550 \
            --out=127.0.0.1:14551 \
            --out=tcpin:0.0.0.0:5761 \
            --console --map

# terminal 3 — web gateway + ngrok tunnels
python3 drone_client.py            # add --no-ngrok for local-only
```

`--out=127.0.0.1:14551` feeds the web gateway; the extra
`--out=tcpin:0.0.0.0:5761` is what the ngrok TCP tunnel forwards to remote
QGroundControl (omit it if you only need the web path).
`drone_client.py` prints the dashboard URL, API key and public tunnel
addresses, then reports vehicle link up/down. Ctrl+C stops the gateway and
tunnels; your simulation keeps running.

## Manual startup (for debugging individual pieces)

Each step in its own terminal, in this order:

| # | What | How |
|---|------|-----|
| 1 | Gazebo world | launch your Gazebo world with the ArduPilot plugin (your existing workflow) |
| 2 | SITL | `scripts/start_sitl.sh` |
| 3 | Router | `scripts/start_router.sh` |
| 4 | Gateway | `scripts/start_server.sh` |
| 5 | Tunnels | `scripts/start_ngrok.sh` |

`start_sitl.sh` runs
`sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out=udp:127.0.0.1:14550`
(plus `--no-extra-ports`, so MAVProxy's default 14550/14551 outputs don't
bypass the router or collide with the gateway port). It does not start the
map or Gazebo.

An optional local GCS can listen on UDP **14552** for debugging.

## Finding your ngrok addresses

`ngrok start --all` prints both endpoints in its terminal UI, e.g.:

```
tcp://5.tcp.eu.ngrok.io:14302    -> localhost:5761
https://abc123.ngrok-free.app    -> http://localhost:8000
```

Or query the local agent API:

```bash
curl -s http://127.0.0.1:4040/api/tunnels | python3 -m json.tool
```

(`public_url` of each tunnel is what you give to remote clients.)

### Remote QGroundControl over the TCP tunnel

In QGroundControl: **Application Settings → Comm Links → Add**

- Type: **TCP**
- Host: `5.tcp.eu.ngrok.io` (your assigned `*.tcp.ngrok.io` host, **without**
  the `tcp://` prefix)
- Port: the assigned port (e.g. `14302`)

Then **Connect**. Mission Planner: same host/port via the TCP connection
option in the top-right connection dropdown.

### Remote dashboard

The gateway serves the dashboard itself at `/`: locally open
**http://127.0.0.1:8000**, remotely open your **https://….ngrok-free.app**
URL — same page either way, with the base-URL field pre-filled. Just paste
the API key and hit **Connect**.

(`client/index.html` is fully static, so you can also copy the file to any
machine and open it directly; then type the base URL yourself.) The
dashboard sends the `ngrok-skip-browser-warning` header so the ngrok
free-plan interstitial doesn't break API calls.

## API

All `/api/*` routes require the `X-API-Key` header; the WebSocket requires
`?api_key=<key>`. Wrong/missing key → HTTP 401 (WebSocket close code 4401).

| Route | Body | Notes |
|-------|------|-------|
| `GET /api/state` | — | latest telemetry snapshot (same dict as the WS pushes) |
| `POST /api/arm` / `/api/disarm` | `{}` | MAV_CMD_COMPONENT_ARM_DISARM, waits for COMMAND_ACK |
| `POST /api/mode` | `{"mode": "GUIDED"}` | any mode name ArduCopter knows |
| `POST /api/takeoff` | `{"altitude": 10}` | **refused (409) unless GUIDED + armed + 3D GPS fix** |
| `POST /api/goto` | `{"lat":…, "lon":…, "alt":…}` | SET_POSITION_TARGET_GLOBAL_INT, relative altitude; requires GUIDED + armed; no ACK exists for this message |
| `POST /api/rtl` / `/api/land` | `{}` | mode change to RTL / LAND |
| `WS /ws/telemetry?api_key=…` | — | `{"type":"state",…}` at 2 Hz, `{"type":"status",…}` (STATUSTEXT) and `{"type":"link",…}` immediately |
| `GET /healthz` | — | unauthenticated liveness probe |

Command error mapping: `409` autopilot rejected (the `detail` contains the
`MAV_RESULT_*` name) or safety guard failed, `503` MAVLink link down,
`504` no COMMAND_ACK within 5 s.

If SITL restarts, the gateway notices heartbeat loss within ~5 s, pushes
`{"type":"link","connected":false}` to all WebSocket clients, reconnects
automatically, and pushes `connected: true` when the heartbeat returns.

## Smoke test

With everything up (steps 1–4; ngrok not needed for a local test):

```bash
python3 scripts/smoke_test.py
```

It runs: link check → wait for 3D fix → GUIDED → arm → takeoff 10 m →
goto 30 m north → RTL, printing PASS/FAIL per step.

The same sequence with curl (`$KEY` is your API key):

```bash
B=http://127.0.0.1:8000; H="X-API-Key: $KEY"; C="Content-Type: application/json"
curl -s -H "$H" $B/api/state | python3 -m json.tool          # check link + GPS fix
curl -s -X POST -H "$H" -H "$C" -d '{"mode":"GUIDED"}' $B/api/mode
curl -s -X POST -H "$H" $B/api/arm
curl -s -X POST -H "$H" -H "$C" -d '{"altitude":10}' $B/api/takeoff
curl -s -X POST -H "$H" -H "$C" \
     -d '{"lat":-35.36302,"lon":149.16523,"alt":10}' $B/api/goto
curl -s -X POST -H "$H" $B/api/rtl
```

(Adjust the goto coordinates to ~30 m from your home location; the default
SITL home is CMAC, -35.363262, 149.165237.)

## Safety & security notes

- Takeoff is refused server-side unless mode is GUIDED, the vehicle is
  armed, and the GPS fix is 3D or better.
- The API key is loaded from `.env` (see `.env.example`) — never hardcoded.
  Anyone with the ngrok TCP address can speak raw MAVLink to the vehicle;
  treat that address like a secret, and stop the tunnel when not in use.
- Set `ALLOWED_ORIGINS` in `.env` to your exact ngrok https URL once known,
  instead of `*`.
