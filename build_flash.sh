#!/usr/bin/env bash

PORT="/dev/ttyUSB0"
FQBN="arduino:avr:nano"
BOARD_OPTIONS="cpu=atmega328old"

arduino-cli lib install AccelStepper
arduino-cli compile --upload --fqbn=${FQBN} --port=${PORT} --board-options=${BOARD_OPTIONS}

