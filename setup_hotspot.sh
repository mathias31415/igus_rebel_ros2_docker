#!/bin/bash

# Set the hotspot name and password
HOTSPOT_NAME="AGV"
HOTSPOT_PASSWORD="agv12345"
HOTSPOT_INTERFACE="wlan0" # on PC: "wlp1s0" #on RPI5: "wlan0"

# Create the hotspot
nmcli device wifi hotspot ifname "$HOTSPOT_INTERFACE" ssid "$HOTSPOT_NAME" password "$HOTSPOT_PASSWORD"
