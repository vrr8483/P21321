#!/usr/bin/env bash
./compile_Arduino.sh
../../../arduino_cli_stuff/arduino-cli upload --fqbn arduino:avr:mega -p /dev/ttyACM0

