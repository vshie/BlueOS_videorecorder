# DropCam - BlueOS Standalone Video Recording Extension

A BlueOS extension that turns a Raspberry Pi 4 or Pi 5 into a standalone, deployable drop-camera recording system. No Navigator autopilot required. Runs on the DeckHand PCB (BR-103953 Rev A), which routes every servo/PWM output through a PCA9685 over I2C, brings the Daly BMS in on the Pi's onboard UART4 (RS-485 via SN65HVD75), and adds a battery-backed MCP7940N RTC so the system clock is correct on boot without internet.

## Features

- **Auto-recording** with configurable delay, duration, and servo movement plans ("recipes")
- **H264 USB camera** recording to power-cut-safe fragmented `.mp4` (ready to play on stop, no remux)
- **Still capture mode** at configurable intervals (0.1s resolution)
- **PCA9685-driven servo outputs** (tilt, lumen light, release, focus, zoom, pan, external, spare) — identical code path on Pi 4 and Pi 5
- **RS-485 Daly BMS** on UART4 with GPIO-driven half-duplex direction control
- **Battery-backed RTC** (MCP7940N) — system clock is set from the RTC at boot and written back once NTP syncs
- **RGB LED** status indicator (WS2812 NeoPixel on GPIO 10)
- **System telemetry** subtitle overlay (CPU temp, voltage, clock, servo position, light level)
- **Disk space guard** — stops recording when < 1 GB free
- **Web interface** with Status, Recipes, and Setup tabs

## Hardware Wiring (DeckHand PCB, BR-103953 Rev A)

Servo/PWM outputs on the PCA9685 (I2C 0x40, J105 header):

| Function | PCA9685 Channel | J105 Pin |
|----------|-----------------|----------|
| Camera Tilt Servo | 0 (TILT) | 1 |
| Lumen Light | 1 (LUMEN) | 4 |
| Release Servo | 2 (RELEASE) | 7 |
| External Servo | 3 (EXTSERVO) | 10 |
| Camera Focus | 4 (FOCUS) | 13 |
| Zoom | 5 (ZOOM) | 16 |
| Pan | 6 (PAN) | 19 |
| Spare | 7 (SPARE) | 22 |

Direct Pi connections:

| Function | Pi GPIO | Pi Pin | Notes |
|----------|---------|--------|-------|
| RGB Status LED (WS2812) | GPIO 10 (SPI MOSI) | Pin 19 | Data line to J104 pin 3 |
| PCA9685 output enable (~OE) | GPIO 4 | Pin 7 | Active LOW; driven low at boot to enable outputs |
| RS-485 UART TX | GPIO 8 (UART4 TX) | Pin 24 | To SN65HVD75 DI |
| RS-485 UART RX | GPIO 9 (UART4 RX) | Pin 21 | From SN65HVD75 RO |
| RS-485 DE / ~RE | GPIO 11 | Pin 23 | HIGH = transmit, LOW = receive |
| I2C-1 SDA | GPIO 2 | Pin 3 | PCA9685 (0x40) + MCP7940N RTC (0x6F) |
| I2C-1 SCL | GPIO 3 | Pin 5 | Same bus |
| Release-shaft rotation sensor | GPIO 20 | Pin 38 | Falling-edge alert, via J104 pin 4 |
| USB Camera | — | /dev/video2 | H264 USB camera |

## Host Prerequisites (BlueOS Pi)

### BlueOS version — must be Bookworm-based (1.5.x or newer)

The extension expects the Linux kernel to support hardware RS-485 direction control on the PL011 UART (the `TIOCSRS485` ioctl). That support landed in Linux 5.14, so the Bullseye-based BlueOS 1.4.x line (kernel 5.10) is **too old** — polls to the Daly BMS will fail because the kernel can't drive DE with tight enough timing and the software fallback can chop bytes off the pack's reply.

Use one of these images from [bluerobotics/BlueOS releases](https://github.com/bluerobotics/BlueOS/releases):

- **Pi 4:** `BlueOS-raspberry-linux-arm-v7-bookworm.zip` (**not** the `-bullseye-pi4` variant on the same release page)
- **Pi 5:** `BlueOS-raspberry-linux-arm64-v8-bookworm-pi5.zip`

Both ship kernel 6.x with full PL011 RS-485 support. After booting the new image run `uname -r` and confirm it starts with `6.` (`5.10.x` means you flashed the wrong variant).

### /boot/config.txt overrides

The extension itself runs as a privileged Docker container, but `/boot/config.txt` is owned by BlueOS (the host OS boots from it before Docker starts). Four DeckHand pin needs collide with defaults BlueOS's `blueos_startup_update` service re-applies on every boot, so hand-editing the file is not enough — the reconciler will revert plain edits on the next reboot.

The repo ships a one-shot script that installs the required overrides **in a form that survives the reconciler**. Copy it to the Pi and run it once (as user `pi`, sudo is invoked internally):

```
scp scripts/apply_deckhand_host_config.sh pi@<pi-ip>:/tmp/
ssh pi@<pi-ip> 'bash /tmp/apply_deckhand_host_config.sh'
ssh pi@<pi-ip> 'sudo reboot'
```

After reboot the `[pi4]` block in `/boot/config.txt` will start with:

```
[pi4]
dtoverlay=uart3-off       # custom - DeckHand: GPIO 4 needed for PCA9685 ~OE
dtoverlay=spi1-3cs-off    # custom - DeckHand: GPIO 20 needed as rotation-sensor input
dtoverlay=uart4,ctsrts    # custom - DeckHand: RTS4 on GPIO 11 for kernel TIOCSRS485 DE
gpio=11,24,25=op,pu,dh
gpio=11=a4,pn             # custom - DeckHand: force GPIO 11 to ALT4 (RTS4)
...
```

You still need I2C bus 1 enabled. On stock BlueOS images the `[all]` block already sets it; if not, add:

```
dtparam=i2c_arm=on        # /dev/i2c-1 for PCA9685 (0x40) + MCP7940N RTC (0x6F)
enable_uart=1
```

Rationale for the overrides applied by the script:

- **`dtoverlay=uart3-off`** is a deliberate no-op overlay name. The Pi firmware silently skips it, so nothing claims GPIO 4/5. But `dtoverlay=uart3-off` still starts with the string `dtoverlay=uart3`, which is what BlueOS's reconciler prefix-matches on, so it never re-adds the real `dtoverlay=uart3` (which would put GPIO 4 into UART3 alt-function and fight the PCA9685 `~OE` line).
- **`dtoverlay=spi1-3cs-off`** applies the same trick to SPI1's 3-chip-select overlay. The real overlay would claim GPIO 16–21 as SPI1 pins, and we need GPIO 20 as an input for the release-shaft rotation sensor.
- **`dtoverlay=uart4,ctsrts`** is a **real** overlay variant: the `,ctsrts` parameter tells the PL011 driver to also grab CTS4 (GPIO 10) and RTS4 (GPIO 11) alongside TXD4/RXD4. This is what lets the kernel drive the SN65HVD75 DE/~RE line for us with hardware-precision timing via `TIOCSRS485` — no Python GPIO toggling, no missed head bytes. The `,ctsrts` suffix is what BlueOS's reconciler prefix-matches on when looking for `dtoverlay=uart4`, so this line satisfies the reconciler while adding what we need.
- **`gpio=11=a4,pn`** is a per-pin override that runs *after* the reconciler's required `gpio=11,24,25=op,pu,dh` line. Pi firmware applies `gpio=` directives top-to-bottom and later per-pin settings override earlier list entries for the same pin, so GPIO 11 boots as ALT4 (RTS4, HIGH-when-idle isn't a concern because the PL011 driver takes ownership immediately after firmware) while 24 and 25 keep their required op,pu,dh state.
- The `# custom` inline comment on the four protected lines triggers `blueos_startup_update.CONFIG_USER_PROTECTION_WORD`, telling the reconciler's filter pass to leave them alone.

The MCP7940N RTC is driven by the extension in Python (no `dtoverlay=i2c-rtc` needed).

Note on ttyAMA naming: the kernel assigns `/dev/ttyAMA<N>` in the order the `dtoverlay=uart<N>` lines load, **not** by hardware UART number. With this DeckHand host config UART4 typically ends up as `/dev/ttyAMA1` on Pi 4 (was `/dev/ttyAMA2` when UART3 was also loaded). The extension auto-discovers UART4 by MMIO address (`serial@7e201800`), so you don't have to guess — set `battery.serial_port` to `"auto"` (the default) and it will find the right tty on either layout.

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

The DeckHand PCB (BR-103953 Rev A) sits on the Pi's 40-pin header. All servo/light/motor outputs come off the PCA9685 (I2C 0x40) via header **J105**, and the Daly BMS connects to header **J108** (RS-485). Refer to the tables in the top of this README for the full pinout — the notes below focus on runtime behaviour:

| Function | Notes |
|----------|-------|
| RGB Status LED (WS2812) | GPIO 10 / SPI MOSI. Single NeoPixel data line to J104 pin 3. |
| Lumen Light | PCA9685 channel 1 (J105 pin 4). 1000-2000 µs servo PWM. Modes: always on, pause points only, or snapshot only. |
| Camera Tilt Servo | PCA9685 channel 0 (J105 pin 1). 1000-2000 µs PWM. Centered at 1500 µs on boot. |
| External Servo | PCA9685 channel 3 (J105 pin 10). 1000-2000 µs PWM. |
| Release Servo | PCA9685 channel 2 (J105 pin 7). Continuous-rotation drive: 1500 µs = stop, 1000 µs = wind, 2000 µs = unwind (frees unit to surface). Held at 1500 µs from boot. |
| Release rotation sensor | GPIO 20 (J104 pin 4). Falling-edge alert via lgpio; 100 ms glitch filter + 250 ms software debounce. |
| Camera Focus | PCA9685 channel 4 (J105 pin 13). 1000-2000 µs servo PWM. |
| Zoom | PCA9685 channel 5 (J105 pin 16). 1000-2000 µs servo PWM. |
| Pan | PCA9685 channel 6 (J105 pin 19). 1000-2000 µs servo PWM. |
| Spare | PCA9685 channel 7 (J105 pin 22). Unused / reserved. |
| Daly BMS (RS-485) | UART4 (GPIO 8 TX / GPIO 9 RX) + GPIO 11 DE/~RE, wired through SN65HVD75. Terminates 120 Ω via SJ101 (unpopulated by default). Connector J108. |
| MCP7940N RTC | I2C 0x6F on the same bus as the PCA9685. Battery-backed by CR1220 on BT101. |
| USB Camera | H264 USB camera (1080p 30fps) at /dev/video2. |

All servo/light/PWM signals share a common ground with the Pi. Servos, lights, and motors require an external 5V power source appropriate for their load; do not power them from the Pi's GPIO header. The PCA9685 outputs are 3.3 V through 220 Ω series resistors (RN101/RN102) — this is enough drive for standard servo signal inputs but not for direct MOSFET gate switching.

### Pi 4 vs Pi 5

Because every servo/PWM output runs off the PCA9685 over I2C, the same driver code path is used on both boards. There is no per-board `dtoverlay` requirement beyond `i2c_arm=on` and `uart4`. The `pigpiod` daemon is still started in the container as a defensive fallback for legacy direct-PWM builds, but is not required on the DeckHand PCB. The active servo backend is reported in `/telemetry` under `gpio_backends` — on the DeckHand PCB you should see `"servo": "pca9685"`.

## 3. LED Status Indicators

When retrieving a deployed camera, the LED tells you exactly what state the system is in:

| LED Pattern | State | What It Means |
|-------------|-------|---------------|
| **Off** | Boot | Extension is starting up and initializing hardware. Wait a few seconds. |
| **Breathing blue** | Idle / Waiting | System is ready. If an auto-start recipe is set, this also shows during the pre-recording delay countdown. |
| **Slow red flash** | Recording | Video or stills capture is in progress. This is the default; recipes can customize the color and blink rate. |
| **Fast yellow flash** | Warning | A problem occurred: recording file not growing, disk full, USB storage disconnected, or scheduler error. Recording may have stopped. |
| **Very fast red flash** (6&nbsp;Hz, full brightness) | Low battery | Battery voltage dropped below the `low_voltage` threshold (default 13.0&nbsp;V). Overrides every other LED state until voltage rises above `clear_voltage` (default 13.2&nbsp;V, 0.2&nbsp;V hysteresis). **Recording is not stopped** — the alarm is advisory only, so recipes keep running while the LED signals the low-power condition. Thresholds configurable in `config.json` under the `"battery"` block. |
| **Slow yellow flash** | Processing | Recording has stopped and the system is applying rotation metadata, transferring files between USB and SD card, or remuxing a legacy TS→MP4 file. Do not remove power or USB drive. |
| **Solid blue** | Complete | A scheduled recording has finished and all processing is done. Safe to power off or retrieve the USB drive. |

**Retrieval lifecycle:** Off → Breathing blue → Slow flash (recording) → Slow yellow flash (processing) → Solid blue (done — safe to retrieve).

Recipe recordings can customize the recording LED color (red, green, blue, yellow, cyan, magenta, white) and blink rate (solid, slow, fast). The other states (idle, warning, processing, complete) are always the same regardless of recipe settings. The low-battery alarm is a special high-priority state that overrides all of the above — recording continues, only the LED indicator changes.

## 4. Time Synchronization

The DeckHand PCB includes a battery-backed MCP7940N RTC on I2C 0x6F. At container startup the extension reads the RTC and, if the year is plausible (>= 2024), sets the Pi system clock via `clock_settime` before anything timestamps a file. Once the OS reports that NTP has synchronised the clock (or the browser sync path completes), a background thread writes the corrected time back to the RTC so a subsequent power cycle boots with an accurate clock.

The RTC's runtime state (present, battery backup, oscillator running, last sync/write time) is surfaced under `rtc` in `/telemetry`.

If the RTC is missing (dev machine, legacy board) or the coin cell is dead the extension falls back to the previous behaviour:

- Connect a phone or laptop to the BlueOS WiFi AP.
- Open this DropCam page in a browser — BlueOS will sync the system clock to your device's time.
- A yellow banner on the Status tab warns when the clock is not synchronized.

## 5. Recording Plans (Recipes)

- Create recipes in the **Recipes** tab to define recording duration, servo movement, and light settings.
- Select a recipe in the **Auto-Start Recipe** dropdown on the Status tab — your selection is saved immediately and persists across reboots and power cycles.
- When the extension starts (on boot), it will wait for the camera, then after the configured delay period, begin recording automatically.
- If the camera is not immediately available after a cold boot, the system retries up to 10 times (30 seconds) before proceeding.
- To disable auto-start, set the dropdown to "None (manual only)".

## 6. Recording Details

- Video is recorded directly to a fragmented **.mp4** during capture. A self-contained fragment is flushed every 5 seconds and the file header is written up front, so the recording stays resilient to power cuts (a crash loses at most the final ~5 s fragment) while being ready to play the moment recording stops — no TS→MP4 remux needed.
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
- **Servo not moving:** Confirm I2C is enabled on the host (`dtparam=i2c_arm=on`) and that `i2cdetect -y 1` shows both `0x40` (PCA9685) and `0x6f` (RTC). Check that `/telemetry` reports `"servo": "pca9685"`. If it shows `sim`, the extension could not open the I2C bus. GPIO 4 (~OE) must be pulled LOW at boot; if it is stuck HIGH, every channel stays high-Z and no servo moves.
- **BMS not detected:** First confirm the kernel is new enough for hardware RS-485 (`uname -r` should start with `6.` — anything `5.10.x` means you're on the old Bullseye BlueOS image and the DE line can't be timed tightly enough). Then confirm `dtoverlay=uart4,ctsrts` is present in `/boot/config.txt` (or `/boot/firmware/config.txt`) — the `,ctsrts` suffix is what puts GPIO 11 in RTS4 alt-function so the kernel can drive DE. With `serial_port: "auto"` (the default) the extension resolves UART4 by MMIO address and logs `"BMS auto-scan: resolved UART4 hardware to /dev/ttyAMA<N>"` at startup, followed by `"RS485 direction control: kernel TIOCSRS485 on /dev/ttyAMA<N>"` once the ioctl is accepted; `/battery` should then report the same `serial_port`. If the TIOCSRS485 log is missing you'll see `"RS485 half-duplex direction control on GPIO 11"` instead — that's the software fallback (works, but drops the first byte of some responses). If you replaced the shield with a USB adapter, keep `serial_port: "auto"` and clear `rs485_de_gpio` to null (the FT232 handles DE itself).
- **RTC time wrong on boot:** Check the CR1220 coin cell (BT101) and `rtc.battery_backup` / `rtc.oscillator_running` in `/telemetry`. `rtc.power_failed` = true means the RTC lost power since the last sync — the extension clears the flag once the system clock is set.
- **LED not lighting:** Ensure the WS2812 data line is on GPIO 10 and shares a ground with the Pi.
- **Recordings empty or corrupt:** Check disk space. Recordings are fragmented .mp4 files that stay playable even if power was lost mid-recording (only the final ~5 s fragment is lost). Legacy .ts files from older builds are auto-remuxed to .mp4 on next start.
- **Extension logs:** View logs from the BlueOS Extension Manager or run `docker logs blueos-videorecorder`.
