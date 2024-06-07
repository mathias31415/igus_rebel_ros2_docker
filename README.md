# IgusReBel

This Repo uses packages from https://github.com/CommonplaceRobotics/iRC_ROS

# Preparations
We use a RaspberryPi 4B with 4GB RAM and installed Ubuntu 22.04 on it using the [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
On Ubuntu we then installed Docker acording to this [tutorial](https://docs.docker.com/engine/install/ubuntu/). We also recomend to install Terminater `sudo apt install terminator`.

Depending on whether you want to use the software on your PC (with amd64 processor) or on a RaspberryPi (with arm64 processor) you have to change the base image in the `Dockerfile` file, by comenting/ uncomenting the following lines:
```
# For PC with amd64: (https://hub.docker.com/r/osrf/ros/tags?page=1&page_size=&name=&ordering=?)
# FROM osrf/ros:$ROS_DISTRO-desktop as base

#For RaspberryPi with arm64: (https://hub.docker.com/r/arm64v8/ros/tags)
FROM arm64v8/ros:$ROS_DISTRO as base
```

# Comands for using the provided Software
### Build Container: 
```
./run.sh
```
### Build ROS packages:
```
colcon build
```
### Source:
```
source install/setup.bash
```

### (optional) enter existing container with another terminal: 
Aktive Docker-Container anzeigen
```
docker ps
```
In Docker-Container hineingehen
```
docker exec -it <container-name> bash
```
Sourcen
```
source install/setup.bash
```

## Without hardware
### Visualize Robot and move joints via joint_state_publisher_gui (opens automaticly)
```
ros2 launch irc_ros_description visualize.launch.py
```
### Visualize Robot on CPR platform and move joints via joint_state_publisher_gui
```
ros2 launch irc_ros_description visualize.launch.py robot_name:=rebel_on_platform
```

### Launch Robot with irc_ros_bringup (without real hardware):
```
ros2 launch irc_ros_bringup rebel.launch.py hardware_protocol:=mock_hardware
```
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```


### Launch Robot on Plattform without real hardware (does not work jet):
```
ros2 launch irc_ros_bringup rebel_on_platform.launch.py hardware_protocol:=mock_hardware use_rqt_robot_steering:=false
```




## Launch Robot with real Hardware
Configure CAN interface (doesn't need to be done in Docker container)
```
sudo ip link set can0 up type can bitrate 500000 restart-ms 1000
```
```
ros2 launch irc_ros_bringup rebel.launch.py rebel_version:=pre launch_dio_controller:=false launch_dashboard_controller:=false
```
```
ros2 launch irc_ros_moveit_config rebel.launch.py gripper:=none launch_dio_controller:=false rebel_version:=pre
```



## TODOs
Fehler bei richtiger Hardware:

[ros2_control_node-3] [INFO] [1717160808.908927550] [iRC_ROS]: Module 0x70: Errors TEMP ESTOP MNE COM LAG LAG DRV OC detected, resetting

- Fehlerausgabe in src/irc_ros_hardware/src/CAN/module.cpp in line 134

- Fehlerdefinition ESTOP, ... in 

- 0x70 und 0x80 in src/irc_ros_description/urdf/robots/igus_rebel_6dof_01.ros2_control.xacro


IDEA:
Try pre version instead of _01 --> Pre version does not have GPIOs
ros2 launch irc_ros_bringup rebel.launch.py rebel_version:=pre launch_dio_controller:=false launch_dashboard_controller:=false
ros2 launch irc_ros_moveit_config rebel.launch.py gripper:=none launch_dio_controller:=false rebel_version:=pre

--> pre version works, but gear ratio seems to



Fehler nach aussortieren Repo:
igus_rebel_user@ubuntu:~/ros2_ws$ ros2 launch irc_ros_moveit_config rebel.launch.py gripper:=none launch_dio_controller:=false rebel_version:=pre
[INFO] [launch]: All log files can be found below /home/igus_rebel_user/.ros/log/2024-06-03-16-07-30-416218-ubuntu-6225
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): executed command failed. Command: /opt/ros/humble/bin/xacro /home/igus_rebel_user/ros2_ws/install/irc_ros_description/share/irc_ros_description/urdf/igus_rebel_6dof.urdf.xacro prefix:= hardware_protocol:=cprcanv2 gripper:=none rebel_version:=pre
Captured stderr output: error: No such file or directory: /home/igus_rebel_user/ros2_ws/install/irc_ros_description/share/irc_ros_description/urdf/grippers/grippers.macro.xacro [Errno 2] No such file or directory: '/home/igus_rebel_user/ros2_ws/install/irc_ros_description/share/irc_ros_description/urdf/grippers/grippers.macro.xacro'
when processing file: /home/igus_rebel_user/ros2_ws/install/irc_ros_description/share/irc_ros_description/urdf/igus_rebel_6dof.urdf.xacro
