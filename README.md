# DropCam - BlueOS Standalone Video Recording Extension

A BlueOS extension that turns a Raspberry Pi 4 into a standalone, deployable drop-camera recording system. No Navigator autopilot required.

## Features

- **Auto-recording** with configurable delay, duration, and servo movement plans ("recipes")
- **H264 USB camera** recording to power-cut-safe `.ts` (MPEG-TS) container
- **Still capture mode** at configurable intervals (0.1s resolution)
- **Camera tilt servo** control (1000-2000 us PWM on GPIO 18)
- **Lumen light** control via servo PWM (GPIO 13)
- **Release servo** continuous-rotation drive for surface recovery (GPIO 12)
- **Focus / zoom / pan / external** auxiliary servo PWM channels (GPIO 20 / 26 / 16 / 19)
- **RGB LED** status indicator (WS2812 NeoPixel on GPIO 10)
- **System telemetry** subtitle overlay (CPU temp, voltage, clock, servo position, light level)
- **Disk space guard** — stops recording when < 1 GB free
- **Web interface** with Status, Recipes, and Setup tabs

## Hardware Wiring

| Component | GPIO | Physical Pin |
|-----------|------|-------------|
| RGB Status LED (WS2812) | GPIO 10 (SPI MOSI) | Pin 19 |
| Camera Tilt Servo | GPIO 18 (PWM, Pin 12) | Pin 12 |
| Lumen Light | GPIO 13 (PWM1) | Pin 33 |
| Release Servo | GPIO 12 | Pin 32 |
| Camera Focus | GPIO 20 | Pin 38 |
| Zoom | GPIO 26 | Pin 37 |
| Pan | GPIO 16 | Pin 36 |
| External Servo | GPIO 19 | Pin 35 |
| USB Camera | — | /dev/video2 |

## Quick Start

1. Flash BlueOS to a Pi 4 microSD card
2. Install the DropCam extension
3. Connect to BlueOS WiFi AP and open the DropCam page to sync time
4. Create a recording recipe or select a default one
5. The camera will auto-start recording after the configured delay on next boot

## Manual Install

To install DropCam manually from the BlueOS Extension Manager, choose **Install from 
Scratch** from the + icon in the lower right of the Installed Extensions page and use:

```text
Image: vshie/blueos-blueos_video_recorder
Tag: dropcam
```

Copy and paste this permissions JSON when BlueOS asks for extension permissions:

```json
{
  "ExposedPorts": {
    "5423/tcp": {}
  },
  "HostConfig": {
    "Binds": [
      "/usr/blueos/extensions/videorecorder:/app/videorecordings",
      "/dev/video2:/dev/video2",
      "/dev/snd:/dev/snd",
      "/dev:/dev"
    ],
    "ExtraHosts": ["host.docker.internal:host-gateway"],
    "PortBindings": {
      "5423/tcp": [
        {
          "HostPort": ""
        }
      ]
    },
    "NetworkMode": "host",
    "Privileged": true
  }
}
```
The other fields don't matter, make them something logical! 
## Note

Connected cameras must have their streams **removed** from the BlueOS Video Streams page so `/dev/video2` is available to the extension.

---

# DropCam Setup Guide

*(This guide is also available in the **Setup** tab of the DropCam web interface.)*

## 1. Initial BlueOS Setup

- Flash BlueOS to a microSD card (32 GB+ recommended, high-endurance).
- Insert the card into your Raspberry Pi 4 and power it on.
- Connect to the **BlueOS WiFi Hotspot** (SSID: `BlueOS (******)`, default password: `blueosap`).
- Navigate to `http://blueos-hotspot.local` or `http://192.168.42.1` to access the BlueOS web interface.
- Install the **DropCam** extension from the Extension Manager.

## 2. Hardware Wiring

| Component | GPIO | Physical Pin | Notes |
|-----------|------|--------------|-------|
| RGB Status LED (WS2812) | GPIO 10 (SPI MOSI) | Pin 19 | Single NeoPixel data line |
| Lumen Light | GPIO 13 (PWM1) | Pin 33 | 1000-2000 µs servo PWM. Modes: always on, pause points only, or snapshot only |
| Camera Tilt Servo | GPIO 18 | Pin 12 | 1000-2000 µs PWM. Centered at 1500 µs on boot. On Pi 5, driven by the RP1 hardware-PWM peripheral (jitter-free) — requires `dtoverlay=pwm-2chan` in config.txt (see Pi 5 notes below) |
| External Servo | GPIO 19 | Pin 35 | 1000-2000 µs PWM. On Pi 5, also hardware-PWM (channel 3) under `dtoverlay=pwm-2chan` |
| Release Servo | GPIO 12 | Pin 32 | Continuous-rotation drive: 1500 µs = stop, 1000 µs = wind, 2000 µs = unwind (frees unit to surface). Held at 1500 µs from boot |
| Camera Focus | GPIO 20 | Pin 38 | 1000-2000 µs servo-style PWM |
| Zoom | GPIO 26 | Pin 37 | 1000-2000 µs servo-style PWM |
| Pan | GPIO 16 | Pin 36 | 1000-2000 µs servo-style PWM |
| USB Camera | /dev/video2 | — | H264 USB camera (1080p 30fps) |

All servo/light/PWM signals share a common ground with the Pi. Servos, lights, and motors require an external 5V power source appropriate for their load; do not power them from the Pi's GPIO header.

### Raspberry Pi 5 — jitter-free servo (hardware PWM)

The Pi 5's RP1 I/O controller cannot do DMA-timed PWM the way `pigpio` did on the Pi 4, so a software-timed servo pulse visibly jitters. To get a clean, jitter-free tilt servo on the Pi 5, the extension drives **GPIO 18** (and the external servo on **GPIO 19**) through the RP1 **hardware-PWM** peripheral. This requires the PWM overlay to be enabled on the host:

1. Edit `/boot/firmware/config.txt` and add a line:

```
dtoverlay=pwm-2chan
```

2. Reboot the Pi.

This maps GPIO 18 → PWM channel 2 and GPIO 19 → PWM channel 3 on the RP1. The extension auto-detects the hardware-PWM chip at startup; if the overlay is missing it falls back to software PWM (functional but jittery). The container must run privileged so `/sys/class/pwm` is writable (already set in the extension permissions). The active servo backend is reported in `/telemetry` under `gpio_backends` (e.g. `rp1-hw-pwm+lgpio`).

The remaining servo-style outputs (release, focus, zoom, pan, light) stay on software PWM, which is fine for their use (release is a continuous-rotation drive; the others are infrequent, low-precision moves).

## 3. LED Status Indicators

When retrieving a deployed camera, the LED tells you exactly what state the system is in:

| LED Pattern | State | What It Means |
|-------------|-------|---------------|
| **Off** | Boot | Extension is starting up and initializing hardware. Wait a few seconds. |
| **Breathing blue** | Idle / Waiting | System is ready. If an auto-start recipe is set, this also shows during the pre-recording delay countdown. |
| **Slow red flash** | Recording | Video or stills capture is in progress. This is the default; recipes can customize the color and blink rate. |
| **Fast yellow flash** | Warning | A problem occurred: recording file not growing, disk full, USB storage disconnected, or scheduler error. Recording may have stopped. |
| **Slow green flash** | Processing | Recording has stopped and the system is remuxing TS→MP4, or transferring files between USB and SD card. Do not remove power or USB drive. |
| **Solid blue** | Complete | A scheduled recording has finished and all processing is done. Safe to power off or retrieve the USB drive. |

**Retrieval lifecycle:** Off → Breathing blue → Slow flash (recording) → Slow green flash (processing) → Solid blue (done — safe to retrieve).

Recipe recordings can customize the recording LED color (red, green, blue, yellow, cyan, magenta, white) and blink rate (solid, slow, fast). The other states (idle, warning, processing, complete) are always the same regardless of recipe settings.

## 4. Time Synchronization

The Raspberry Pi does not have a real-time clock (RTC). Without internet access, the system time resets on each boot. To synchronize the clock:

- Connect a phone or laptop to the BlueOS WiFi AP.
- Open this DropCam page in a browser — the system clock will be synchronized to your device's time automatically by BlueOS.
- A yellow banner on the Status tab warns when the clock is not synchronized.

## 5. Recording Plans (Recipes)

- Create recipes in the **Recipes** tab to define recording duration, servo movement, and light settings.
- Select a recipe in the **Auto-Start Recipe** dropdown on the Status tab — your selection is saved immediately and persists across reboots and power cycles.
- When the extension starts (on boot), it will wait for the camera, then after the configured delay period, begin recording automatically.
- If the camera is not immediately available after a cold boot, the system retries up to 10 times (30 seconds) before proceeding.
- To disable auto-start, set the dropdown to "None (manual only)".

## 6. Recording Details

- Video is recorded as MPEG-TS during capture (resilient to power cuts), then automatically remuxed to **.mp4** when recording stops for maximum player compatibility.
- Still capture mode saves JPEG frames at the configured interval.
- Subtitle files (.ass) are generated alongside video recordings with system telemetry data.
- Recording stops automatically if disk space drops below **1 GB**.
- Recipe duration is capped at **32 hours** for now (until field battery/runtime limits are confirmed). Storage, power, and SD endurance still apply.

## 7. File Management

- Recorded files are stored at `/usr/blueos/extensions/videorecorder/` on the Pi.
- Download files from the **Status** tab or use the BlueOS File Manager.
- The BlueOS file browser is available at `http://blueos.local:7777/files/extensions/videorecorder`.

## 8. Troubleshooting

- **Camera not detected:** Ensure the USB camera is connected and appears as `/dev/video2`. Replug and restart the extension.
- **Servo not moving:** Verify wiring and that the pigpio daemon is running inside the container. Check extension logs.
- **LED not lighting:** Ensure the WS2812 data line is on GPIO 10 and shares a ground with the Pi.
- **Recordings empty or corrupt:** Check disk space. If power was lost during recording, a .ts file may remain (not yet remuxed to .mp4) but is still playable.
- **Extension logs:** View logs from the BlueOS Extension Manager or run `docker logs blueos-videorecorder`.
