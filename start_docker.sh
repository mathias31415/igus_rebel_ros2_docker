#!/bin/sh
ROS_DISTRO=humble
SRC_CONTAINER=/home/igus_rebel_user/ros2_ws/src
SRC_HOST="$(pwd)/src"

echo "Run Container"

docker run \
  --name igusrebel \
  -it \
  --privileged \
  --net=host \
  --env-file .env\
  -v /dev:/dev \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY="$DISPLAY" \
  --restart always \
  igus-rebel/ros-render:"$ROS_DISTRO"

# restart always enables automatic container startup on boot