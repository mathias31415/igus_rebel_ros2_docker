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

## Launch robot with mock hardware --> funktioniert so nicht mehr, siehe unten
```
ros2 launch irc_ros_moveit_config rebel.launch.py gripper:=none launch_dio_controller:=false rebel_version:=01_without_dio hardware_protocol:=mock_hardware
```

## Launch robot with real hardware --> funktioniert so nicht mehr, siehe unten
Configure CAN interface (doesn't need to be done in Docker container)
```
sudo ip link set can0 up type can bitrate 500000 restart-ms 1000
```
```
ros2 launch irc_ros_moveit_config rebel.launch.py gripper:=none launch_dio_controller:=false rebel_version:=01_without_dio
```


## Note E-STop
As we use the open source version without a controller, the emergency stop is not connected to the CAN bus. The emergency stop interrupts the supply voltage of the axes (5V logic voltage is retained). After resetting the emergency stop, the robot is not automatically activated. The ros software must therefore be relaunched manually.

#####################################################################################################################
## Launch after complete system integration

1) clone the repo on a raspery py
2) build the image with ```build_docker.sh```
3) start the conteiner ONCE with ```start_docker.sh``` ---> note: if you have done this ONCE, the contaienr will start at every new rasperry boot automatically and all ROS2 nodes are up!
4) you can access the contanier from a new attached terminal
5) for visualization run ```ros2 launch irc_ros_bringup rviz.launch.py```


### ACHTUNG: currently with mock_hardware in autostart --> modify dockerfile for real hardware
## Fixes/ Problems
Could not load the Qt platform plugin "xcb" in "" even though it was found. --> ```xhost +local:docker```

## TODOs
- verschiedene planer in RVIZ --> bei UR oder diy_robotics schauen ---> CHECK
- moveit clients und python interface ---> CHECK
- 2 separate launch files für rviz ---> CHECK
- run in build und start skript aufteilen für autostart ---> CHECK
- .env Datei für ROS Domain hinzufügen ---> CHECK
- Autostart der ROS-Application auf RaspberryPi beim hochfahren ---> CHECK
- CAN Adapter automatisch einrichten, damit befehl nicht jedes mal ins Terminal eingegeben werden muss ---> CHECK
- Raspy Reboot Button über Python Skript abfragen und das in Autostart packen ---> CHECK
    - installieren lokal: sudo apt-get install -y python3-rpi-gpio ---> import RPi.GPIO as GPIO ---> GEHT NICHT DA RPI 5 ANDEREN GPIP CHIP HAT  
    - statdessen verwenden: sudo apt-get install python3-gpiozero ---> from gpiozero import Button
- Raspy WIFO Hotspot öffnen ---> CHECK
- xhost baim Autostart Problem lösen ---> nicht mehr notwendig
