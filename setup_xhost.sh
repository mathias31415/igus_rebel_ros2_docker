#!/bin/bash

# configure xhost for docker container to enable display access for GUI applications (eg rviz) --> AFTER display manager is started !!!
DISPLAY=:0
sleep 10
xhost +local:docker