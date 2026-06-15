import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    controllers_share = get_package_share_directory("legged_controllers")

    gait_file = LaunchConfiguration("gait_file").perform(context)
    if not gait_file:
        gait_file = os.path.join(controllers_share, "config", robot_type, "gait.info")

    reference_file = LaunchConfiguration("reference_file").perform(context)
    if not reference_file:
        reference_file = os.path.join(controllers_share, "config", robot_type, "reference.info")

    task_file = LaunchConfiguration("task_file").perform(context)
    if not task_file:
        task_file = os.path.join(controllers_share, "config", robot_type, "task.info")

    target_publisher = Node(
        package="legged_controllers",
        executable="legged_target_trajectories_publisher",
        name="legged_robot_target",
        output="screen",
        parameters=[
            {"referenceFile": reference_file},
            {"taskFile": task_file},
        ],
    )

    return [
        target_publisher,
        Node(
            package="legged_p1_hw",
            executable="p1_gait_dds_bridge",
            name="p1_gait_dds_bridge",
            output="screen",
            parameters=[
                {"dds_domain": ParameterValue(LaunchConfiguration("dds_domain"), value_type=int)},
                {"dds_gait_topic": LaunchConfiguration("dds_gait_topic")},
                {"gait_file": gait_file},
                {"gait_id_mapping": LaunchConfiguration("gait_id_mapping")},
                {"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")},
                {"emergency_stop_topic": LaunchConfiguration("emergency_stop_topic")},
                {"mode_schedule_topic": LaunchConfiguration("mode_schedule_topic")},
                {"posture_command_topic": LaunchConfiguration("posture_command_topic")},
                {"stand_gait_id": ParameterValue(LaunchConfiguration("stand_gait_id"), value_type=int)},
                {"lie_down_gait_id": ParameterValue(LaunchConfiguration("lie_down_gait_id"), value_type=int)},
                {"mpc_reference_file": reference_file},
                {"emergency_reset_mode_schedule": ParameterValue(LaunchConfiguration("emergency_reset_mode_schedule"), value_type=bool)},
                {"robot_name": LaunchConfiguration("robot_name")},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="p1"),
            DeclareLaunchArgument("dds_domain", default_value="6"),
            DeclareLaunchArgument("dds_gait_topic", default_value="p1_gait_command"),
            DeclareLaunchArgument("gait_file", default_value=""),
            DeclareLaunchArgument("gait_id_mapping", default_value="stance,trot,standing_trot,pace,static_walk,lie_down"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("emergency_stop_topic", default_value="/p1_emergency_stop"),
            DeclareLaunchArgument("mode_schedule_topic", default_value="/legged_robot_mpc_mode_schedule"),
            DeclareLaunchArgument("posture_command_topic", default_value="/legged_robot_posture_command"),
            DeclareLaunchArgument("reference_file", default_value=""),
            DeclareLaunchArgument("task_file", default_value=""),
            DeclareLaunchArgument("stand_gait_id", default_value="0"),
            DeclareLaunchArgument("lie_down_gait_id", default_value="5"),
            DeclareLaunchArgument(
                "emergency_reset_mode_schedule",
                default_value="false",
                description="true: publish default mode schedule from reference on emergency stop; false: only stop command output.",
            ),
            DeclareLaunchArgument("robot_name", default_value="legged_robot"),
            OpaqueFunction(function=launch_setup),
        ]
    )
