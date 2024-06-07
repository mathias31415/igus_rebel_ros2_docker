#!/bin/bash
##############################################################################
##                   Build the image, using dev.Dockerfile                  ##
##############################################################################
ROS_DISTRO=humble

uid=$(eval "id -u")
gid=$(eval "id -g")

#pass some arguments and settings to the dev.Dockerfile while building the image (dev.Dockerfile)
#name of the image builded here: igus-rebel/ros-render:"$ROS_DISTRO":"ROS-Distribution eg humble"
#dont use cached data to clone up-to date repos all the time

#--no-cache \
docker build \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg UID="$uid" \
  --build-arg GID="$gid" \
  -f Dockerfile \
  -t igus-rebel/ros-render:"$ROS_DISTRO" .


##############################################################################
##                            Run the container                             ##
##############################################################################
SRC_CONTAINER=/home/igus_rebel_user/ros2_ws/src
SRC_HOST="$(pwd)/src"

docker run \
  --name igusrebel\
  --rm \
  -it \
  --privileged \
  --net=host \
  -v /dev:/dev \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY="$DISPLAY" \
  igus-rebel/ros-render:"$ROS_DISTRO" bash

# display and network access is already passed to the container
