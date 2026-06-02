import os
import sys

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    task_file = LaunchConfiguration("task_file").perform(context)
    reference_file = LaunchConfiguration("reference_file").perform(context)

    description_share = get_package_share_directory("legged_description")
    gazebo_share = get_package_share_directory("legged_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    controllers_share = get_package_share_directory("legged_controllers")
    _launch_dir = os.path.join(controllers_share, "launch")
    if _launch_dir not in sys.path:
        sys.path.insert(0, _launch_dir)
    from generated_paths import get_generated_dir, resolve_task_file

    task_file = resolve_task_file(task_file, robot_type)

    robot_xacro = os.path.join(description_share, "urdf", "robot.xacro")
    world_file = os.path.join(gazebo_share, "worlds", "empty_world.world")
    generated_dir = get_generated_dir()
    generated_urdf = os.path.join(generated_dir, f"{robot_type}.urdf")
    controller_params_file = os.path.join(generated_dir, f"{robot_type}_gazebo_controllers.yaml")

    with open(controller_params_file, "w", encoding="utf-8") as params:
        yaml.safe_dump(
            {
                "legged_controller": {
                    "ros__parameters": {
                        "urdfFile": generated_urdf,
                        "taskFile": task_file,
                        "referenceFile": reference_file,
                        "imuName": "base_imu",
                    }
                },
                "legged_cheater_controller": {
                    "ros__parameters": {
                        "urdfFile": generated_urdf,
                        "taskFile": task_file,
                        "referenceFile": reference_file,
                        "imuName": "base_imu",
                    }
                },
            },
            params,
        )

    robot_doc = xacro.process_file(
        robot_xacro,
        mappings={
            "robot_type": robot_type,
            "hardware_plugin": "legged_gazebo/LeggedHWSim",
            "power_limit": "4",
            "contact_threshold": "40",
            "delay": "0.009",
            "delay_cycles": "9",
            "controller_params_file": controller_params_file,
        },
    )
    robot_description_xml = robot_doc.toprettyxml(indent="  ")
    with open(generated_urdf, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_description_xml)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")),
        launch_arguments={
            "world": world_file,
            "init": "true",
            "factory": "true",
            "force_system": "true",
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", robot_type, "-z", "0.4", "-timeout", "180"],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
        ],
        output="screen",
    )

    legged_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "legged_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
        ],
        output="screen",
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_spawner, legged_controller_spawner],
        )
    )

    disable_model_database = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    return [disable_model_database, gzserver, gzclient, robot_state_publisher, spawn_entity, spawn_controllers]


def generate_launch_description():
    controllers_share = get_package_share_directory("legged_controllers")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="go1"),
            DeclareLaunchArgument("task_file", default_value=os.path.join(controllers_share, "config", "go1", "task.info")),
            DeclareLaunchArgument("reference_file", default_value=os.path.join(controllers_share, "config", "go1", "reference.info")),
            DeclareLaunchArgument("gui", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
