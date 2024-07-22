#!/bin/bash

# configure xhost for docker container to enable display access for GUI applications (eg rviz) --> AFTER display manager is started !!!
export DISPLAY=:0
xhost local:docker