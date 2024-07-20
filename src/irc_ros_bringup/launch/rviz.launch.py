from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    moveit_package = "sew_and_igus_moveit_config"
    bringup_package = "irc_ros_bringup"

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file_name",
            default_value='rebel_moveit.rviz',
            description="filemane of the rviz config file",
        )
    )
    rviz_config_file_name = LaunchConfiguration("rviz_config_file_name")
    rviz_file = PathJoinSubstitution([FindPackageShare(bringup_package), "rviz", rviz_config_file_name])

    robot_description_kinematics_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_package),
            "config",
            "kinematics.yaml",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[
            robot_description_kinematics_file,  # needed for interactive marker on the tcp
        ],
        arguments=["-d", rviz_file],
    )

    nodes_to_start = [rviz_node]

    return LaunchDescription(declared_arguments + nodes_to_start)