import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    task_file = LaunchConfiguration("task_file").perform(context)
    reference_file = LaunchConfiguration("reference_file").perform(context)
    hardware_plugin = LaunchConfiguration("hardware_plugin").perform(context)
    power_limit = LaunchConfiguration("power_limit").perform(context)
    contact_threshold = LaunchConfiguration("contact_threshold").perform(context)

    description_share = get_package_share_directory("legged_unitree_description")
    controllers_share = get_package_share_directory("legged_controllers")

    robot_xacro = os.path.join(description_share, "urdf", "robot.xacro")
    controllers_yaml = os.path.join(controllers_share, "config", "controllers.yaml")
    generated_urdf = os.path.join("/tmp", f"legged_unitree_{robot_type}_ros2_control.urdf")

    robot_doc = xacro.process_file(
        robot_xacro,
        mappings={
            "robot_type": robot_type,
            "hardware_plugin": hardware_plugin,
            "power_limit": power_limit,
            "contact_threshold": contact_threshold,
        },
    )
    robot_description_xml = robot_doc.toprettyxml(indent="  ")
    with open(generated_urdf, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_description_xml)

    robot_description = {"robot_description": robot_description_xml}

    controller_params = {
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
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml, controller_params],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    legged_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["legged_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return [
        robot_state_publisher,
        ros2_control_node,
        joint_state_spawner,
        legged_controller_spawner,
    ]


def generate_launch_description():
    controllers_share = get_package_share_directory("legged_controllers")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="go1"),
            DeclareLaunchArgument("task_file", default_value=os.path.join(controllers_share, "config", "go1", "task.info")),
            DeclareLaunchArgument("reference_file", default_value=os.path.join(controllers_share, "config", "go1", "reference.info")),
            DeclareLaunchArgument("hardware_plugin", default_value="legged_unitree_hw/UnitreeHW_3_8_0"),
            DeclareLaunchArgument("power_limit", default_value="4"),
            DeclareLaunchArgument("contact_threshold", default_value="40"),
            OpaqueFunction(function=launch_setup),
        ]
    )
