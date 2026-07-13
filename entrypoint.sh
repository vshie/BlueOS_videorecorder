#!/bin/bash
# DeckHand PCB path: all servo/PWM outputs go through the PCA9685 over I2C
# (smbus2) and the rotation sensor uses lgpio alerts, so no pigpio daemon
# is required at runtime. Still start pigpiod in the background as a soft
# fallback for legacy direct-PWM boards where make_servo_backend() picks
# the PigpioServoBackend path -- it exits harmlessly if the socket is
# already bound or the daemon can't attach.
pigpiod 2>/dev/null || true
sleep 0.2
exec python3 -u /app/main.py
