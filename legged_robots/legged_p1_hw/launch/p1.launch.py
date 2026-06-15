import os
import sys

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    emergency_stop_topic = LaunchConfiguration("emergency_stop_topic").perform(context)
    dds_joint_order = LaunchConfiguration("dds_joint_order").perform(context)
    contact_estimation_method = LaunchConfiguration("contact_estimation_method").perform(context)
    contact_force_threshold = LaunchConfiguration("contact_force_threshold").perform(context)
    current_to_torque_scale = LaunchConfiguration("current_to_torque_scale").perform(context)
    current_to_torque_offset = LaunchConfiguration("current_to_torque_offset").perform(context)
    feedforward_torque_slew_rate = LaunchConfiguration("feedforward_torque_slew_rate").perform(context)
    max_feedforward_torque = LaunchConfiguration("max_feedforward_torque").perform(context)
    start_gait_bridge = LaunchConfiguration("start_gait_bridge")
    start_target_publisher = LaunchConfiguration("start_target_publisher")
    dds_gait_topic = LaunchConfiguration("dds_gait_topic").perform(context)
    gait_file = LaunchConfiguration("gait_file").perform(context)
    gait_id_mapping = LaunchConfiguration("gait_id_mapping").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    mode_schedule_topic = LaunchConfiguration("mode_schedule_topic").perform(context)
    posture_command_topic = LaunchConfiguration("posture_command_topic").perform(context)
    stand_gait_id = LaunchConfiguration("stand_gait_id").perform(context)
    lie_down_gait_id = LaunchConfiguration("lie_down_gait_id").perform(context)

    controllers_share = get_package_share_directory("legged_controllers")
    _launch_dir = os.path.join(controllers_share, "launch")
    if _launch_dir not in sys.path:
        sys.path.insert(0, _launch_dir)
    from generated_paths import get_generated_dir, resolve_task_file

    task_file = resolve_task_file(task_file, robot_type)
    if not gait_file:
        gait_file = os.path.join(controllers_share, "config", robot_type, "gait.info")

    description_share = get_package_share_directory("legged_description")
    controllers_yaml = os.path.join(controllers_share, "config", "controllers.yaml")
    generated_dir = get_generated_dir()
    generated_urdf = os.path.join(generated_dir, f"{robot_type}_p1_hw.urdf")
    controller_params_file = os.path.join(generated_dir, f"{robot_type}_p1_hw_controllers.yaml")

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
            "emergency_stop_topic": emergency_stop_topic,
            "dds_joint_order": dds_joint_order,
            "contact_estimation_method": contact_estimation_method,
            "contact_force_threshold": contact_force_threshold,
            "current_to_torque_scale": current_to_torque_scale,
            "current_to_torque_offset": current_to_torque_offset,
            "feedforward_torque_slew_rate": feedforward_torque_slew_rate,
            "max_feedforward_torque": max_feedforward_torque,
        },
    )
    robot_description_xml = robot_doc.toprettyxml(indent="  ")
    with open(generated_urdf, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_description_xml)

    robot_description = {"robot_description": robot_description_xml}
    with open(controller_params_file, "w", encoding="utf-8") as params:
        yaml.safe_dump(
            {
                "legged_controller": {
                    "ros__parameters": {
                        "urdfFile": generated_urdf,
                        "taskFile": task_file,
                        "referenceFile": reference_file,
                        "imuName": "base_imu",
                        "emergencyStopTopic": emergency_stop_topic,
                    }
                },
                "legged_cheater_controller": {
                    "ros__parameters": {
                        "urdfFile": generated_urdf,
                        "taskFile": task_file,
                        "referenceFile": reference_file,
                        "imuName": "base_imu",
                        "emergencyStopTopic": emergency_stop_topic,
                    }
                },
            },
            params,
        )

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
        parameters=[robot_description, controllers_yaml, controller_params_file],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout","120","--service-call-timeout","120", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    legged_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["legged_controller", "--controller-manager-timeout","120","--service-call-timeout","120", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    target_publisher = Node(
        package="legged_controllers",
        executable="legged_target_trajectories_publisher",
        name="legged_robot_target",
        output="screen",
        condition=IfCondition(start_target_publisher),
        parameters=[
            {"referenceFile": reference_file},
            {"taskFile": task_file},
        ],
    )

    gait_bridge = Node(
        package="legged_p1_hw",
        executable="p1_gait_dds_bridge",
        name="p1_gait_dds_bridge",
        output="screen",
        condition=IfCondition(start_gait_bridge),
        parameters=[
            {"dds_domain": int(dds_domain)},
            {"dds_gait_topic": dds_gait_topic},
            {"gait_file": gait_file},
            {"gait_id_mapping": gait_id_mapping},
            {"cmd_vel_topic": cmd_vel_topic},
            {"emergency_stop_topic": emergency_stop_topic},
            {"mode_schedule_topic": mode_schedule_topic},
            {"posture_command_topic": posture_command_topic},
            {"stand_gait_id": int(stand_gait_id)},
            {"lie_down_gait_id": int(lie_down_gait_id)},
            {"mpc_reference_file": reference_file},
            {"emergency_reset_mode_schedule": ParameterValue(LaunchConfiguration("emergency_reset_mode_schedule"), value_type=bool)},
            {"robot_name": "legged_robot"},
        ],
    )

    return [
        robot_state_publisher,
        ros2_control_node,
        joint_state_spawner,
        legged_controller_spawner,
        target_publisher,
        gait_bridge,
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
            DeclareLaunchArgument("emergency_stop_topic", default_value="/p1_emergency_stop"),
            DeclareLaunchArgument("dds_joint_order", default_value="LF_HAA,LF_HFE,LF_KFE,LH_HAA,LH_HFE,LH_KFE,RF_HAA,RF_HFE,RF_KFE,RH_HAA,RH_HFE,RH_KFE"),
            DeclareLaunchArgument("contact_estimation_method", default_value="jacobian"),
            DeclareLaunchArgument("contact_force_threshold", default_value="40.0"),
            DeclareLaunchArgument("current_to_torque_scale", default_value="1"),
            DeclareLaunchArgument("current_to_torque_offset", default_value="0.0"),
            DeclareLaunchArgument("feedforward_torque_slew_rate", default_value="150.0"),
            DeclareLaunchArgument("max_feedforward_torque", default_value="0.0"),
            DeclareLaunchArgument("start_gait_bridge", default_value="true"),
            DeclareLaunchArgument(
                "start_target_publisher",
                default_value="true",
                description="启动 legged_target_trajectories_publisher，将 /cmd_vel 转为 MPC target（与仿真一致，trot 行走需要）。",
            ),
            DeclareLaunchArgument("dds_gait_topic", default_value="p1_gait_command"),
            DeclareLaunchArgument("gait_file", default_value=""),
            DeclareLaunchArgument("gait_id_mapping", default_value="stance,trot,standing_trot,pace,static_walk,lie_down"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("mode_schedule_topic", default_value="/legged_robot_mpc_mode_schedule"),
            DeclareLaunchArgument("posture_command_topic", default_value="/legged_robot_posture_command"),
            DeclareLaunchArgument("stand_gait_id", default_value="0"),
            DeclareLaunchArgument("lie_down_gait_id", default_value="5"),
            DeclareLaunchArgument(
                "emergency_reset_mode_schedule",
                default_value="false",
                description="true: 急停时发布 reference 里 defaultModeSequenceTemplate 重置 MPC；false(默认): 不发布 mode_schedule。",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
