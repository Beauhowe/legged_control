import os
import sys

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
    contact_threshold = LaunchConfiguration("contact_threshold").perform(context)
    command_mode = LaunchConfiguration("command_mode").perform(context)
    dds_domain = LaunchConfiguration("dds_domain").perform(context)
    dds_state_topic = LaunchConfiguration("dds_state_topic").perform(context)
    dds_imu_topic = LaunchConfiguration("dds_imu_topic").perform(context)
    dds_command_topic = LaunchConfiguration("dds_command_topic").perform(context)
    dds_joint_order = LaunchConfiguration("dds_joint_order").perform(context)
    contact_estimation_method = LaunchConfiguration("contact_estimation_method").perform(context)
    contact_force_threshold = LaunchConfiguration("contact_force_threshold").perform(context)
    current_to_torque_scale = LaunchConfiguration("current_to_torque_scale").perform(context)
    current_to_torque_offset = LaunchConfiguration("current_to_torque_offset").perform(context)

    controllers_share = get_package_share_directory("legged_controllers")
    _launch_dir = os.path.join(controllers_share, "launch")
    if _launch_dir not in sys.path:
        sys.path.insert(0, _launch_dir)
    from generated_paths import get_generated_dir, resolve_task_file

    task_file = resolve_task_file(task_file, robot_type)

    description_share = get_package_share_directory("legged_description")
    controllers_yaml = os.path.join(controllers_share, "config", "controllers.yaml")
    generated_dir = get_generated_dir()
    generated_urdf = os.path.join(generated_dir, f"{robot_type}_p1_hw.urdf")

    robot_xacro = os.path.join(description_share, "urdf", "p1", "robot.xacro")
    robot_doc = xacro.process_file(
        robot_xacro,
        mappings={
            "robot_type": robot_type,
            "hardware_plugin": hardware_plugin,
            "contact_threshold": contact_threshold,
            "command_mode": command_mode,
            "dds_domain": dds_domain,
            "dds_state_topic": dds_state_topic,
            "dds_imu_topic": dds_imu_topic,
            "dds_command_topic": dds_command_topic,
            "dds_joint_order": dds_joint_order,
            "contact_estimation_method": contact_estimation_method,
            "contact_force_threshold": contact_force_threshold,
            "current_to_torque_scale": current_to_torque_scale,
            "current_to_torque_offset": current_to_torque_offset,
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
            DeclareLaunchArgument("robot_type", default_value="p1"),
            DeclareLaunchArgument("task_file", default_value=os.path.join(controllers_share, "config", "p1", "task.info")),
            DeclareLaunchArgument("reference_file", default_value=os.path.join(controllers_share, "config", "p1", "reference.info")),
            DeclareLaunchArgument("hardware_plugin", default_value="legged_p1_hw/P1HW"),
            DeclareLaunchArgument("contact_threshold", default_value="40"),
            DeclareLaunchArgument("command_mode", default_value="10"),
            DeclareLaunchArgument("dds_domain", default_value="0"),
            DeclareLaunchArgument("dds_state_topic", default_value="p1_motor_state"),
            DeclareLaunchArgument("dds_imu_topic", default_value="p1_imu"),
            DeclareLaunchArgument("dds_command_topic", default_value="p1_motor_cmd"),
            DeclareLaunchArgument("dds_joint_order", default_value="LF_HAA,LF_HFE,LF_KFE,LH_HAA,LH_HFE,LH_KFE,RF_HAA,RF_HFE,RF_KFE,RH_HAA,RH_HFE,RH_KFE"),
            DeclareLaunchArgument("contact_estimation_method", default_value="current"),
            DeclareLaunchArgument("contact_force_threshold", default_value="40.0"),
            DeclareLaunchArgument("current_to_torque_scale", default_value="1.0"),
            DeclareLaunchArgument("current_to_torque_offset", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
