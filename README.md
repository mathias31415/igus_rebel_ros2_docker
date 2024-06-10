# igus_rebel_ros2_docker

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
Show active docker containers
```
docker ps
```
Enter your docker container
```
docker exec -it <container-name> bash
```
source
```
source install/setup.bash
```

## Launch robot with mock hardware
```
ros2 launch irc_ros_moveit_config rebel.launch.py gripper:=none launch_dio_controller:=false rebel_version:=01_without_dio hardware_protocol:=mock_hardware
```

## Launch robot with real hardware
Configure CAN interface (doesn't need to be done in Docker container)
```
sudo ip link set can0 up type can bitrate 500000 restart-ms 1000
```
```
ros2 launch irc_ros_moveit_config rebel.launch.py gripper:=none launch_dio_controller:=false rebel_version:=01_without_dio
```


## Note E-STop
As we use the open source version without a controller, the emergency stop is not connected to the CAN bus. The emergency stop interrupts the supply voltage of the axes (5V logic voltage is retained). After resetting the emergency stop, the robot is not automatically activated. The ros software must therefore be relaunched manually.


## TODOs
- verschiedene planer in RVIZ --> bei UR oder diy_robotics schauen
- Autostart der ROS-Application auf RaspberryPi beim hochfahren
- CAN Adapter automatisch einrichten, damit befehl nicht jedes mal ins Terminal eingegeben werden muss
