import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    use_gazebo = LaunchConfiguration("use_gazebo")
    gui = LaunchConfiguration("gui")

    description_share = get_package_share_directory("lr_pro_description")
    gazebo_share = get_package_share_directory("legged_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    robot_xacro = os.path.join(description_share, "robot.xacro")
    world_file = os.path.join(gazebo_share, "worlds", "empty_world.world")

    robot_doc = xacro.process_file(
        robot_xacro,
        mappings={"robot_type": robot_type},
    )
    robot_description_xml = robot_doc.toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_description_xml}

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")),
        launch_arguments={
            "world": world_file,
            "init": "true",
            "factory": "true",
            "force_system": "true",
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")),
        condition=IfCondition(AndSubstitution(use_gazebo, gui)),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", robot_type, "-z", "0.75"],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    return [
        joint_state_publisher,
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="lr_pro"),
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
