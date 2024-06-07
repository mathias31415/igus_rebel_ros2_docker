##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble
# For PC with amd64: (https://hub.docker.com/r/osrf/ros/tags?page=1&page_size=&name=&ordering=?
FROM osrf/ros:$ROS_DISTRO-desktop as base

#For RaspberryPi with arm64: (https://hub.docker.com/r/arm64v8/ros/tags)
#FROM arm64v8/ros:$ROS_DISTRO as base

# Configure DDS
COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

# Create user with root privilege
ARG USER=igus_rebel_user
ARG UID=1000
ARG GID=1000
ENV USER=$USER
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID --shell $(which bash) $USER 

#install ROS2 packages
USER root
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-xacro
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-joint-state-publisher-gui
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-controller-interface 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-controller-manager 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-hardware-interface 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-pluginlib 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rclcpp
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rclcpp-lifecycle
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros2-control
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros2-controllers
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rosidl-default-generators
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-std-srvs
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-moveit
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-moveit-common
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-moveit-servo
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-xacro
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-joint-trajectory-controller
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-joint-state-broadcaster
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-sensor-msgs-py
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-joy* 
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rqt-controller-manager
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-navigation2
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-nav2-bringup
#RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-turtlebot3*
USER $USER

# Setup workpace
USER $USER
RUN mkdir -p /home/$USER/ros2_ws
WORKDIR /home/$USER/ros2_ws


#install dependencies for python interface
USER root
RUN apt-get update && apt-get install -y pip
# to show usb devices with lsusb
RUN apt-get update && apt-get install -y usbutils  
# to configure the CAN interface
RUN apt-get update && apt-get install -y iproute2

RUN pip install scipy

# Build the workspace 
# RUN cd /home/$USER/ros2_ws/ && \
#     . /opt/ros/$ROS_DISTRO/setup.sh && \
#     colcon build --packages-select irc_ros_msgs && \
#     colcon build --packages-select irc_ros_description && \
#     colcon build --packages-select irc_ros_moveit_config && \
#     colcon build --packages-select irc_ros_hardware && \
#     colcon build --packages-select irc_ros_controllers && \
#     colcon build --packages-select irc_ros_bringup

# wichtig damit rviz geöffnet werden kann
USER $USER  
#USER root

