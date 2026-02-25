# Webcam Video Feed in Mission Planner HUD

This guide explains how to display a live webcam feed inside the Mission Planner
HUD (Heads-Up Display) using GStreamer. It covers the full setup for:

- Ubuntu 22.04 (what was done here, step-by-step)
- Other Ubuntu versions
- Windows (native installation)

---

## How It Works

Mission Planner has a built-in GStreamer-based video receiver. When Mission
Planner starts, it opens listeners on several UDP ports waiting for incoming
video streams. The default port for H.264 video is **5600**.

To display a webcam in the HUD, you run a GStreamer pipeline on the same machine
that captures the webcam, encodes it as H.264, wraps it in RTP packets, and
sends it to `127.0.0.1:5600`. Mission Planner receives those packets and renders
the video inside the HUD automatically.

```
/dev/video0  →  gst-launch  →  RTP/H264 UDP  →  port 5600  →  Mission Planner HUD
```

---

## Part 1 — Ubuntu 22.04 (step-by-step of what was done)

### 1.1 Prerequisites

```bash
# Verify your webcam is detected
ls /dev/video*          # should show /dev/video0 (or video1, etc.)

# Install GStreamer and the required plugins
sudo apt update
sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

# Verify all needed elements are present
gst-inspect-1.0 v4l2src    # webcam source
gst-inspect-1.0 x264enc    # H.264 encoder
gst-inspect-1.0 rtph264pay # RTP packetiser
```

### 1.2 Fix libSkiaSharp (required for Mission Planner on Linux)

Mission Planner ships a `libSkiaSharp.so` compiled against **musl libc**, but
Ubuntu uses **glibc**. Loading a musl binary inside a glibc process (Mono)
causes a fatal crash. You must replace it with the glibc build.

```bash
# Step 1 — download the glibc build from NuGet (SkiaSharp 2.88.1)
cd /tmp
curl -L -o skiasharp.nupkg \
  "https://www.nuget.org/api/v2/package/SkiaSharp.NativeAssets.Linux/2.88.1"

# Step 2 — extract the correct .so
mkdir -p /tmp/skiasharp
unzip -q skiasharp.nupkg \
  "runtimes/linux-x64/native/libSkiaSharp.so" \
  -d /tmp/skiasharp/

# Step 3 — back up the original and replace it
cp ~/MissionPlanner/x64/libSkiaSharp.so \
   ~/MissionPlanner/x64/libSkiaSharp.so.bak
cp /tmp/skiasharp/runtimes/linux-x64/native/libSkiaSharp.so \
   ~/MissionPlanner/x64/libSkiaSharp.so

# Step 4 — verify all dependencies are satisfied (no "not found" lines)
ldd ~/MissionPlanner/x64/libSkiaSharp.so
```

> **Version note:** The NuGet version must match the managed `SkiaSharp.dll`
> inside Mission Planner. Check with:
> ```bash
> monodis --assembly ~/MissionPlanner/SkiaSharp.dll | grep Version
> ```
> The assembly version `2.88.x.x` requires native API `88.1` or higher.
> NuGet versions 2.88.1 through 2.88.8 all provide native API 88.1 and work.

### 1.3 Launch Mission Planner

```bash
cd ~/MissionPlanner
DISPLAY=:1 nohup mono MissionPlanner.exe >> /tmp/mp.log 2>&1 &
disown $!
```

Wait ~15 seconds for it to fully load. You can monitor startup with:

```bash
tail -f /tmp/mp.log
# Ready when you see: "show FlightData... Done"
```

### 1.4 Start the webcam stream

Open a second terminal and run:

```bash
gst-launch-1.0 -q \
  v4l2src device=/dev/video0 \
  ! video/x-raw,width=640,height=480,framerate=30/1 \
  ! videoconvert \
  ! x264enc tune=zerolatency bitrate=2000 key-int-max=30 \
  ! video/x-h264,profile=baseline \
  ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host=127.0.0.1 port=5600
```

As soon as data arrives on port 5600, Mission Planner displays it in the HUD
**automatically** — no extra click needed.

To run it in the background permanently:

```bash
nohup gst-launch-1.0 -q \
  v4l2src device=/dev/video0 \
  ! video/x-raw,width=640,height=480,framerate=30/1 \
  ! videoconvert \
  ! x264enc tune=zerolatency bitrate=2000 key-int-max=30 \
  ! video/x-h264,profile=baseline \
  ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host=127.0.0.1 port=5600 >> /tmp/gst.log 2>&1 &
disown $!
```

### 1.5 HUD Video menu reference

Right-clicking the HUD opens a **Video** submenu:

| Menu item | What it does |
|---|---|
| **Set GStreamer Source** | Enter a custom pipeline and start it immediately |
| **GStreamer Stop** | Stops the active pipeline (use sparingly — see note below) |
| **Start Camera** | Opens a system camera picker (DirectShow/V4L2) |
| **Set MJPEG Source** | Point Mission Planner at an MJPEG HTTP stream |
| **HereLink Video** | Shortcut for HereLink drone video feed |
| **Record HUD to AVI** | Start recording the HUD to a file |
| **Stop Record** | Stop the AVI recording |

> ⚠️ **Important:** Clicking **GStreamer Stop** kills Mission Planner's internal
> receiver. The external `gst-launch` pipeline you started keeps running, but
> Mission Planner won't display it. To restore the feed **without restarting**:
>
> 1. Right-click HUD → Video → **Set GStreamer Source**
> 2. Paste this pipeline and click OK:
>
> ```
> udpsrc port=5600 buffer-size=90000 ! application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false
> ```

---

## Part 2 — Other Ubuntu versions

The steps are identical. The only things that may differ are package names and
the SkiaSharp fix.

### Package names by Ubuntu version

| Ubuntu | GStreamer install command |
|---|---|
| 20.04 (Focal) | Same `apt install` as 22.04 above |
| 22.04 (Jammy) | Same as above (tested) |
| 24.04 (Noble) | Same as above — `gstreamer1.0-libav` may be `gstreamer1.0-vaapi` |

### SkiaSharp version check

Every time you install a new Mission Planner build, check whether the bundled
`libSkiaSharp.so` is musl or glibc before launching:

```bash
ldd ~/MissionPlanner/x64/libSkiaSharp.so
```

- If output shows `libc.musl-x86_64.so.1 => not found` → follow Section 1.2
- If all lines show a path (e.g. `libc.so.6 => /lib/...`) → no fix needed

### Mono version

Mission Planner stable (1.3.83) runs on Mono 6.8.x (the default in Ubuntu
repos). The beta build (1.3.9551+) requires Mono 6.12.x or newer, available
from the official Mono repository:

```bash
# Add mono-project.com repo (required for beta Mission Planner)
sudo gpg --homedir /tmp --no-default-keyring \
  --keyring /usr/share/keyrings/mono-official-archive-keyring.gpg \
  --keyserver hkp://keyserver.ubuntu.com:80 \
  --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF

echo "deb [signed-by=/usr/share/keyrings/mono-official-archive-keyring.gpg] \
  https://download.mono-project.com/repo/ubuntu stable-focal main" \
  | sudo tee /etc/apt/sources.list.d/mono-official-stable.list

sudo apt update && sudo apt install -y mono-complete
```

---

## Part 3 — Windows

On Windows, Mission Planner is a native .exe — no Mono, no libSkiaSharp fix
needed. There are two methods to show a webcam in the HUD.

### Method A — Direct camera capture (simplest)

Mission Planner on Windows can capture a webcam directly using DirectShow:

1. Open Mission Planner → **Flight Data** tab
2. Right-click the HUD → **Video** → **Start Camera**
3. A dialog lists all detected cameras — select your webcam and click OK

The feed appears in the HUD immediately. This requires no GStreamer installation.

### Method B — GStreamer pipeline (same as Linux, more control)

Use this if you need to adjust resolution, framerate, bitrate, or stream from a
remote camera.

**Step 1 — Install GStreamer for Windows**

Download the 64-bit MSVC runtime installer from:
https://gstreamer.freedesktop.org/download/

Install both:
- `gstreamer-1.0-msvc-x86_64-*.msi` (runtime)
- `gstreamer-1.0-devel-msvc-x86_64-*.msi` (development, for `gst-launch-1.0.exe`)

Add GStreamer to your PATH (the installer offers this option).

**Step 2 — Start the webcam stream**

Open a Command Prompt or PowerShell and run:

```powershell
gst-launch-1.0.exe -q ^
  ksvideosrc device-index=0 ^
  ! video/x-raw,width=640,height=480,framerate=30/1 ^
  ! videoconvert ^
  ! x264enc tune=zerolatency bitrate=2000 key-int-max=30 ^
  ! video/x-h264,profile=baseline ^
  ! rtph264pay pt=96 config-interval=1 ^
  ! udpsink host=127.0.0.1 port=5600
```

> `ksvideosrc` is the Windows webcam source (Kernel Streaming).
> Use `device-index=0` for the first camera, `1` for the second, etc.

**Step 3 — Mission Planner will display it automatically**

Just like on Linux, the video appears in the HUD as soon as data arrives on
port 5600. If it doesn't appear automatically, use:

Right-click HUD → Video → **Set GStreamer Source** → paste:
```
udpsrc port=5600 buffer-size=90000 ! application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false
```

### Notes specific to Windows

- **Firewall:** If Windows Firewall blocks UDP 5600, add an inbound rule:
  `netsh advfirewall firewall add rule name="MP Video" protocol=UDP dir=in localport=5600 action=allow`
- **GStreamer PATH:** If `gst-launch-1.0.exe` is not found, add
  `C:\gstreamer\1.0\msvc_x86_64\bin` to your system PATH.
- **Multiple cameras:** Change `device-index=0` to `1`, `2`, etc. to select a
  different camera.

---

## Quick-start cheat sheet

### Linux

```bash
# 1 — start Mission Planner
cd ~/MissionPlanner
DISPLAY=:1 nohup mono MissionPlanner.exe >> /tmp/mp.log 2>&1 & disown $!

# 2 — start webcam stream
nohup gst-launch-1.0 -q v4l2src device=/dev/video0 \
  ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert \
  ! x264enc tune=zerolatency bitrate=2000 key-int-max=30 \
  ! video/x-h264,profile=baseline ! rtph264pay pt=96 config-interval=1 \
  ! udpsink host=127.0.0.1 port=5600 >> /tmp/gst.log 2>&1 & disown $!

# 3 — video appears in HUD automatically

# to restore video after GStreamer Stop:
# right-click HUD → Video → Set GStreamer Source → paste the udpsrc pipeline
```

### Windows

```powershell
# Method A (no GStreamer needed)
# Right-click HUD → Video → Start Camera → select webcam

# Method B (GStreamer)
gst-launch-1.0.exe -q ksvideosrc device-index=0 ^
  ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ^
  ! x264enc tune=zerolatency bitrate=2000 key-int-max=30 ^
  ! video/x-h264,profile=baseline ! rtph264pay pt=96 config-interval=1 ^
  ! udpsink host=127.0.0.1 port=5600
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `DllNotFoundException: libSkiaSharp` | musl-compiled `.so` on glibc system | Follow Section 1.2 |
| `libc.musl-x86_64.so.1` in `ldd` output | musl `.so` loaded, causes crash | Follow Section 1.2 |
| `native libSkiaSharp version incompatible` | Wrong NuGet version downloaded | Check `monodis --assembly SkiaSharp.dll` and pick matching NuGet version |
| Video stopped, won't come back | Clicked GStreamer Stop | Right-click HUD → Video → Set GStreamer Source → paste udpsrc pipeline |
| No video on startup | GStreamer stream not running | Start `gst-launch-1.0` pipeline before or after launching Mission Planner |
| Black HUD, no video | Wrong port or pipeline | Confirm `gst-launch` is sending to port 5600 and Mission Planner is listening |
| `v4l2src` error | Wrong device path | Check `ls /dev/video*` and update `device=/dev/videoX` accordingly |
| Mission Planner crashes immediately | Multiple issues possible | Check `/tmp/mp.log` for the first `ERROR` or `FATAL` line |
