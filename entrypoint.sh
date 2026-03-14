#!/bin/bash
# Start pigpio daemon for servo/light PWM, then launch Flask app
pigpiod 2>/dev/null || true
sleep 0.5
exec python3 -u /app/main.py
