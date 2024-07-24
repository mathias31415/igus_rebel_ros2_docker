# igus_rebel_ros2_docker

## Repo Structure and ROS2 Packages:
This is a core breakdown of the official documentation to the FE-Project of Hannes Bornamann, Mathias Fuhrer and Robin Wolf at the Hochschule Karlsruhe (SS24). This readme should guide users to get familiar with the igus rebel 6DoF robot arm and its capabilities. Deeper informations and background knowledge is provided in the offical documentation only.
The docker container provided inside this repo runs on the RasperryPi inside the robot base and handles all tasks of the robot control. Moreover the same container can be used by a user to control the robot via RVIZ-GUI or self written high level control scripts. The users PC has just to be logged in a the hosted local network.

## Usage of containerized enviroments with Docker:
### Introduction:
The usage of Docker is a common Praxis when working with ROS2. The biggest advantage of devbeloping a ROS ecosystem inside a containerized enviroment is that it can be used independent of the host hardware.
Everyone who wants to use this repo has just to clone the repo from GitHub to the local disk and run the Dockerfile. No ROS2 installation on the host machine necessary !
All needed ROS2-Packages are installed and set up by default when running the Dockerfile. Moreover the network setup for the ROS2 Node communication over topics with fast-RTPS defined in the dds_profile.xml is done automatically.

To use the providede Dockerfile the following prequisities are required on your host machine:

- Ubuntu 22.04 (NOT in a Virtual Machine !) https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview
- Working installation of Docker Engine https://docs.docker.com/engine/install/ubuntu/

### Usage - Preperations:
We use a RaspberryPi 5 with 8GB RAM and installed Ubuntu 24.04 on it using the [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
On Ubuntu we then installed docker acording to this [tutorial](https://docs.docker.com/engine/install/ubuntu/). We also recomend to install terminater `sudo apt install terminator`.

Depending on whether you want to use the software on your PC (with amd64 processor) or on a RaspberryPi (with arm64 processor) you have to change the base image in the `Dockerfile` file, by comenting/ uncomenting the following lines:
```shell
# For PC with amd64: (https://hub.docker.com/r/osrf/ros/tags?page=1&page_size=&name=&ordering=?)
FROM osrf/ros:$ROS_DISTRO-desktop as base

#For RaspberryPi with arm64: (https://hub.docker.com/r/arm64v8/ros/tags)
FROM arm64v8/ros:$ROS_DISTRO as base
```

## Provided ROS2-Packages:
### Description Packages

The description packages provide the full kinematic definition and CAD data of our robot system. In ROS2 the kinematics of the robot is defined in a URDF model. The URDF model can be structred by using the xacro package and define sub-macros which are all put together in the main URDF.
Moreover some tags regarding the hardware-communication with ROS2-Control and some tags are specified in these packages too.  
**Note:** The AGV part of the robot system is managed in a seperate container and can not be controlled by using this container only. Here, its just a static model for collision avoidence.

- irc_ros_description: kinematic description and ros2 control definitions of the Igus ReBel Arm (cloned from https://github.com/CommonplaceRobotics/iRC_ROS/tree/humble/irc_ros_description)
- sew_agv_description: kinematic description of the sew-maxo-mts AGV at Hochschule Karlsruhe
- sew_and_igus_description: combined kinematic description of the arm and the AGV to one united robot with more DoF.

### Bringup and Control Packages

These packages provide further functionalities for hardware communication.

- irc_ros_bringup: handles the correct bringup of all needed ROS2 nodes and has to be launched by the user to start the system (see How To)(cloned from https://github.com/CommonplaceRobotics/iRC_ROS/tree/humble/irc_ros_bringup).
- irc_ros_controllers: defines the dio-controller to use the dio ports on the igus robot arm. These are currently not available, because the HKA bought the uopen-source version of the robot. On this version no hardware dio ports are available (cloned from https://github.com/CommonplaceRobotics/iRC_ROS/tree/humble/irc_ros_controllers).   
- irc_ros_hardware: defines the controller with a hardware interface for CAN communication between the container and the different axis modules (cloned from https://github.com/CommonplaceRobotics/iRC_ROS/tree/humble/irc_ros_hardware)
- irc_ros_msgs: defines several custom messages and service types needed for communication purposes when the robot system is active (cloned from https://github.com/CommonplaceRobotics/iRC_ROS/tree/humble/irc_ros_msgs)
  
### Motion Planning and Application Packages

These packages provide all functionalities regarding the robots motion planning and the user interface.

- trac-ik: contains the sourcecode and plugin definition for an advanced IK-solver which is used by MoveIt in our configuration (cloned from https://bitbucket.org/traclabs/trac_ik/src/rolling-devel/)
- sew_and_igus_moveit_config: configuration of the motion planning capabilities with MoveIt2 embedded in ROS2 and handling of the needed nodes to plan and execute trajectories. The handling of the occupancy-map collision gemetries extracted from the cmaera point-cloud is also done here.
- moveit_wrapper: provides service servers which can control the motion planning capabilities through the C++ move_group interface
- igus_moveit_clients: provides a python class handling clients which connect to the servers from the warpper package. The user can call the class methods from a supervised python file to provide a simple approach of using MoveIt motion planning capabilities.
- robot_application: this package provides a simple programmable interface for the ROS2 ecosystem. The user can implement his own supervised control logic in python conde to move the robot system in simulation and real world by calling the metods provided in the clinet classes (see How To)

## How-To operate the real robot
The following graphic shows all of the needed hardware buttons and switches.  
<img src="img/switches_back.png" alt="drawing" width="1200"/>  
<img src="img/switches_robot.png" alt="drawing" width="1200"/>

### mount the igus on the AGV

**Note:** This should only be done by experienced and authorized personal with deactivated AGV!

1) remove the steel plate on the top of the AGV by 4 bolts
2) slide the steel plate to the back of the AGV to get access to the electrics in front of the batteries
3) place the plate with the igus and the cabinet on top of the agv
4) push the power and ethernet cable to the hub mounted on the AGVs steel plate
5) connect the ethernet cable to the switch in the AGVs electrics drawer (red cable):  
   <img src="img/ethernet.png" alt="drawing" width="1200"/>
6) connect the power cable to the electrical load hub:  
   <img src="img/powercable.png" alt="drawing" width="1200"/>
7) mount the AGVs steel plate witn 2 M8 bolts in the front and mount the robot plate with 3 M8 bolts in the back
8) turn on the agv and the robot to check functionality

### start the robot

1) turn on the AGV by pressing and holding the green and blue button for a few seconds
2) check, if the emergency stop is not pushed, if so then pull the emergency-stop out to enable the robot to start
3) turn on the main switch
4) wait until the robot has completely booted. You should hear a quiet "klick" when the brakes release and you should notice a new local network named "AGV" is hosted by the robot.
5) clone the repo on your private user PC and build and start the docker container (**Iportant:** during these steps, you are not allowed to be logged in to the robots private network!) 
6) after the container has started, kill the opened terminal with ctrl+C, the container keeps running in the background
7) connect your user PC to the robots local network named "AGV"
8) kill and restart the docker container to pass the new network settings with ```docker kill igusrebel``` and ```docker start igusrebel```
9) open another terminal window and connect to the container with ```docker exec -it igusrebel bash```
10) source the workspace with ```source install/setup.bash```
    
**Note:** steps 5 and 6 are only recommendet for the first usage, for any further usages these steps can be skipped

### restart the robot after an emergency stop

1) pull the emergency-stop out to enable the robot to start
2) reboot the roboot with the small red button on the robot base
3) go back to 4) above

### move the robot with RVIZ

1) move to the newly opened terminal window from 10) above
2) launch RVIZ with: ```ros2 launch irc_ros_bringup rviz.launch.py``` 
3) wait until RVIZ has opened and the robot is completely visible
4) try to move the robot with the interactive marker on the tcp or in the window at the bottom right (MotionPlanning/Joints). The oragne goal state should has moved to your recommendet pose.
5) PTP: plan and execute the trajectory with the "plan and execute" button in the window at the bottom right (MotionPlanning/Planning).   
   LIN: click the checkbox "USe Cartesian Path" and then press the "plan and execute" button in the window at the bottom right (MotionPlanning/Planning).

The robot shold move now. (Hint: you can change planning algorithms and parameters to experience the different robot behaviors)

### run own control scripts

Its recommendet to do these tasks in a terminal window parallel to using RVIZ, because all motions can be checked here.

1)  connect a new, second terminal to the running container on your user PC with: ```docker exec -it igusrebel bash``` and source inside the new terminal with ```source install/setup.bash```
2)  execute your supervised control script: ```ros2 run robot_application <your_control_script>```
3)  you should see the robots motion in RVIZ and your codes terminal feedback prints in the second terminal
   
## How-To write own control scripts

1) open the repo in VSCode or a similar programming IDE on your user PC
2) navigate to: src/robot_application/robot_application
3) add a new python file
4) add the new python file entrypoint to the setup.py file similar to the provided example
5) Write your own control script with the provided methods (shown below). You also can take the provided example as a base and develop your code in there.

```python
# class variables
self.home_position = [0.0,0.0,0.0,0.0,0.0,0.0]  # [joint1, joint2, joint3, joint4, joint5, joint6]

# class methods
def get_transform(self, from_frame_rel, to_frame_rel, affine=True):
    """
    string from_frame_rel: name of the source frame frame to transform from (child)
    string to_frame_rel: name of the target frame to transform into (parent)

    Returns:
    --------
    Affine: transformation matrix from robot base to target position and orientation (4x4 numpy array, can directly passed in motion planning)
    or
    geometry_msgs/TransformStamped: transformation between the two frames if affine=False
    
    """
def reset_planning_group(self, planning_group):
    """
    string planning_group: name of the planning group of the arm (default: igus_6dof)

    Returns
    -------
    bool success

    """
def setVelocity(self, fraction):
    """
    float: velocity caling coeffinet relative to joint speed limits (0.01 ... 0.5 recommendet)

    Returns
    -------
    bool success

    """
def home(self):
    """

    Returns
    -------
    bool success

    """
def ptp(self, pose: Affine):
    """
    cartesian goal pose: affine transformation martix format (can be converted from x,y,z, r,p,y or x,y,z and quaternion)

    Returns
    -------
    bool success

    """
def ptp_joint(self, joint_positions: List[float]):
    """
    joint space goal pose: list of goal joint states (rad)

    Returns
    -------
    bool success

    """
def lin(self, pose: Affine):
    """
    cartesian goal pose: affine transformation matrix format (can be converted from x,y,z, r,p,y or x,y,z and quaternion)

    Returns
    -------
    bool success

    """
```