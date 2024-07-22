#!/bin/bash

# configure xhost for docker container to enable display access for GUI applications (eg rviz) --> AFTER display manager is started !!!
DISPLAY=:0
TIMEOUT=60  # Maximum time to wait for X server (in seconds)
INTERVAL=5   # Interval between checks (in seconds)

for ((i=0; i<$TIMEOUT; i+=$INTERVAL)); do
  if xdpyinfo -display $DISPLAY >/dev/null 2>&1; then
    echo "X server is available"
    xhost +local:docker
    exit 0
  fi
  sleep $INTERVAL
done

echo "X server did not become available in time"
exit 1