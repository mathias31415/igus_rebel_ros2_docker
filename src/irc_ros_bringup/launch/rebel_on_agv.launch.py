from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    igus_moveit_package = "sew_and_igus_moveit_config"
    bringup_package = "irc_ros_bringup"

    declared_arguments = []
    # agv launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='sew_',
            description="Prefix for the links and joints from tha SEW AGV",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value='true',
            description="launch the drivers that connect to the real agv hardware via IP",  # not used in the igus repo, agv is just a collision object here
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description="use the fake hardware on the agv, because in this repo the agv is only used for collision avoidance, agv control is done in a seperate container",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Set to "true" if you want to use the gazebo clock, set to "fasle" if you use real hardware.'
        )
    )

    # rebel launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="The namespace to use for all nodes started by this launch file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_protocol",
            default_value="cprcanv2",
            choices=["mock_hardware", "gazebo", "cprcanv2"],
            description="select the hardware protocol which should be used by ros2 control resource manager",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="none",
            choices=["none", "ext_dio_gripper"],
            description="Which gripper to attach to the flange",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='igus_',
            description="Prefix for the links and joints from tha Igus Rebel 6DoF robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rebel_version",
            default_value="01_without_dio",
            choices=["pre", "00", "01", "01_without_dio"],
            description="Which version of the igus ReBeL to use",
        )
    )

    # general arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit",
            default_value='true',
            description="set to true if you want to launch moveit for the igus arm",
        )
    )



    #init launch arguments, transfer to variables
    tf_prefix = LaunchConfiguration("tf_prefix")
    generate_ros2_control_tag = LaunchConfiguration('generate_ros2_control_tag') 
    use_sim_time = LaunchConfiguration('use_sim_time')   
    prefix = LaunchConfiguration('prefix') 
    hardware_protocol = LaunchConfiguration('hardware_protocol') 
    namespace = LaunchConfiguration('namespace')
    gripper = LaunchConfiguration('gripper')
    rebel_version = LaunchConfiguration('rebel_version')
    launch_moveit = LaunchConfiguration('launch_moveit')




    #launch additional hardware controllers
    irc_ros_bringup_launch_dir = PathJoinSubstitution(
        [
            FindPackageShare(bringup_package),
            "launch",
        ]
    )
    additional_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/additional_controllers.launch.py"]
        ),
    )

    #launch igus driver with gazebo ros2 control hardware interface and moveit if launch argument is set to true
    load_igus = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(igus_moveit_package), 'launch']), "/controller_and_moveit.launch.py"]),
            condition=IfCondition(launch_moveit),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "generate_ros2_control_tag": generate_ros2_control_tag,
                "namespace": namespace,
                "prefix": prefix,
                "tf_prefix": tf_prefix,
                "gripper": gripper,
                "hardware_protocol": hardware_protocol,
                "rebel_version": rebel_version,
            }.items(),
    )



    nodes_to_start = [
        # additional_controllers,       --> currently not supported due to the open source hardware version of the igus
        load_igus,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
