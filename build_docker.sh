#!/bin/bash
ROS_DISTRO=humble

echo "Build Container"

uid=$(eval "id -u")
gid=$(eval "id -g")

#--no-cache \
docker build \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg UID="$uid" \
  --build-arg GID="$gid" \
  -f Dockerfile \
  -t igus-rebel/ros-render:"$ROS_DISTRO" .
