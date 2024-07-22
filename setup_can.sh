#!/bin/bash

# configure usb to can bridge to igus robot
sudo ip link set can0 up type can bitrate 500000 restart-ms 1000
