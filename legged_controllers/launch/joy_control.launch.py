"""一键启动：joy_node、legged_target_trajectories_publisher、joy_control（cmd_vel + 步态组合键）。"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *_args, **_kwargs):
    from launch_ros.actions import Node

    pkg = get_package_share_directory("legged_controllers")

    legacy_joy_dev = LaunchConfiguration("joy_dev").perform(context).strip()
    if legacy_joy_dev:
        print(
            "[joy_control.launch.py] WARN: 已忽略无效参数 joy_dev:="
            + legacy_joy_dev
            + "（ROS2 joy_node 不能用设备路径）。请改用 joy_device_id:=0 或 joy_device_name:=..."
        )

    # ROS 2 joy_node 使用 SDL：device_name 是「手柄名称字符串」，不能填 /dev/input/js0。
    # 未设置 joy_device_name 时用 device_id（SDL 枚举下标，通常 0 第一个、1 第二个）。
    joy_device_id = int(LaunchConfiguration("joy_device_id").perform(context))
    joy_device_name = LaunchConfiguration("joy_device_name").perform(context).strip()
    start_joy_node = LaunchConfiguration("start_joy_node")
    robot_type = LaunchConfiguration("robot_type").perform(context)
    joy_topic = LaunchConfiguration("joy_topic").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    teleop_config = LaunchConfiguration("teleop_config").perform(context)
    gait_mappings = LaunchConfiguration("gait_mappings_file").perform(context)

    task_file = LaunchConfiguration("task_file").perform(context)
    reference_file = LaunchConfiguration("reference_file").perform(context)
    stand_reference_file = LaunchConfiguration("stand_reference_file").perform(context)
    lie_down_reference_file = LaunchConfiguration("lie_down_reference_file").perform(context)
    gait_file = LaunchConfiguration("gait_command_file").perform(context)

    if not task_file:
        task_file = os.path.join(pkg, "config", robot_type, "task.info")
    _launch_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
    if _launch_dir not in sys.path:
        sys.path.insert(0, _launch_dir)
    from generated_paths import resolve_task_file

    task_file = resolve_task_file(task_file, robot_type)
    if not reference_file:
        reference_file = os.path.join(pkg, "config", robot_type, "reference.info")
    if not stand_reference_file:
        stand_reference_file = os.path.join(pkg, "config", robot_type, "reference.info")
    if not lie_down_reference_file:
        lie_down_reference_file = os.path.join(pkg, "config", robot_type, "reference_lie_down.info")
    if not gait_file:
        gait_file = os.path.join(pkg, "config", robot_type, "gait.info")

    joy_params = {
        "device_id": joy_device_id,
        "deadzone": 0.001,
        "autorepeat_rate": 10.0,
        "coalesce_interval_ms": 5,
    }
    if joy_device_name:
        joy_params["device_name"] = joy_device_name

    return [
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            condition=IfCondition(start_joy_node),
            parameters=[joy_params],
        ),
        Node(
            package="legged_controllers",
            executable="legged_target_trajectories_publisher",
            name="legged_robot_target",
            output="screen",
            parameters=[
                {"referenceFile": reference_file},
                {"taskFile": task_file},
            ],
        ),
        Node(
            package="legged_controllers",
            executable="joy_control",
            name="joy_control",
            output="screen",
            parameters=[
                {"teleop_config_file": teleop_config},
                {"gait_mappings_file": gait_mappings},
                {"gait_command_file": gait_file},
                {"locomotion_gaits": LaunchConfiguration("locomotion_gaits")},
                {"joy_topic": joy_topic},
                {"cmd_vel_topic": cmd_vel_topic},
                {"stand_reference_file": stand_reference_file},
                {"lie_down_reference_file": lie_down_reference_file},
            ],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("legged_controllers")

    default_teleop = os.path.join(pkg, "config", "joy.yaml")
    default_mappings = os.path.join(pkg, "config", "joy_gait_mappings.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_device_id",
                default_value="0",
                description="SDL 下手柄序号（0=第一个）。不要用 /dev/input/js* 路径。",
            ),
            DeclareLaunchArgument(
                "joy_device_name",
                default_value="",
                description="可选：SDL 报告的手柄名称（与 device_id 二选一逻辑见 joy 源码）；留空则用 device_id。",
            ),
            DeclareLaunchArgument(
                "joy_dev",
                default_value="",
                description="已废弃（仅占位）。请勿传 /dev/input/js*；请用 joy_device_id。",
            ),
            DeclareLaunchArgument(
                "start_joy_node",
                default_value="true",
                description="true: 启动 USB/SDL joy_node；false: 使用已有 /joy，例如 UnitreeHW 发布的遥控器话题。",
            ),
            DeclareLaunchArgument("joy_topic", default_value="joy"),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="",
                description="可选：覆盖 joy.yaml 中的 teleop.walk.topic_name；留空时使用 joy.yaml。",
            ),
            DeclareLaunchArgument("robot_type", default_value="go1"),
            DeclareLaunchArgument("teleop_config", default_value=default_teleop),
            DeclareLaunchArgument("gait_mappings_file", default_value=default_mappings),
            DeclareLaunchArgument("task_file", default_value=""),
            DeclareLaunchArgument("reference_file", default_value=""),
            DeclareLaunchArgument("stand_reference_file", default_value=""),
            DeclareLaunchArgument("lie_down_reference_file", default_value=""),
            DeclareLaunchArgument("gait_command_file", default_value=""),
            DeclareLaunchArgument("locomotion_gaits", default_value="trot,static_walk"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
