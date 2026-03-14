# DropCam - BlueOS Standalone Video Recording Extension

A BlueOS extension that turns a Raspberry Pi 4 into a standalone, deployable drop-camera recording system. No Navigator autopilot required.

## Features

- **Auto-recording** with configurable delay, duration, and servo movement plans ("recipes")
- **H264 USB camera** recording to power-cut-safe `.ts` (MPEG-TS) container
- **Still capture mode** at configurable intervals (0.1s resolution)
- **Camera tilt servo** control (1000-2000 us PWM on GPIO 21)
- **Lumen light** control via servo PWM (GPIO 13)
- **RGB LED** status indicator (WS2812 NeoPixel on GPIO 10)
- **System telemetry** subtitle overlay (CPU temp, voltage, clock, servo position, light level)
- **Disk space guard** — stops recording when < 1 GB free
- **Web interface** with Status, Recipes, and Setup tabs

## Hardware Wiring

| Component | GPIO | Physical Pin |
|-----------|------|-------------|
| RGB Status LED (WS2812) | GPIO 10 (SPI MOSI) | Pin 19 |
| Camera Tilt Servo | GPIO 21 | Pin 40 |
| Lumen Light | GPIO 13 (PWM1) | Pin 33 |
| USB Camera | — | /dev/video2 |

## Quick Start

1. Flash BlueOS to a Pi 4 microSD card
2. Install the DropCam extension
3. Connect to BlueOS WiFi AP and open the DropCam page to sync time
4. Create a recording recipe or select a default one
5. The camera will auto-start recording after the configured delay on next boot

## Note

Connected cameras must have their streams **removed** from the BlueOS Video Streams page so `/dev/video2` is available to the extension.
