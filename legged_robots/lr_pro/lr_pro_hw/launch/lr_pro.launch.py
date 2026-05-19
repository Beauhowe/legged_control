import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 真机/硬件接口启动入口。
# 这个 launch 不启动 Gazebo，而是启动 robot_state_publisher、ros2_control_node 和 legged_controller。
# 默认 hardware_plugin 为 lr_pro_hw/LrProHW，对应 lr_pro_hw_plugins.xml 中导出的插件名。
def launch_setup(context, *args, **kwargs):
    # 从 launch 参数读取机器人类型、控制任务文件、硬件插件和 ros2_control 传参。
    robot_type = LaunchConfiguration("robot_type").perform(context)
    task_file = LaunchConfiguration("task_file").perform(context)
    reference_file = LaunchConfiguration("reference_file").perform(context)
    hardware_plugin = LaunchConfiguration("hardware_plugin").perform(context)
    power_limit = LaunchConfiguration("power_limit").perform(context)
    contact_threshold = LaunchConfiguration("contact_threshold").perform(context)
    delay = LaunchConfiguration("delay").perform(context)

    description_share = get_package_share_directory("lr_pro_description")
    controllers_share = get_package_share_directory("legged_controllers")

    # 使用 lr_pro_description 的顶层 robot.xacro 生成带 ros2_control 的 URDF。
    robot_xacro = os.path.join(description_share, "robot.xacro")
    controllers_yaml = os.path.join(controllers_share, "config", "controllers.yaml")
    generated_urdf = os.path.join("/tmp", f"lr_pro_{robot_type}_ros2_control.urdf")

    robot_doc = xacro.process_file(
        robot_xacro,
        mappings={
            "robot_type": robot_type,
            "hardware_plugin": hardware_plugin,
            "power_limit": power_limit,
            "contact_threshold": contact_threshold,
            "delay": delay,
        },
    )
    robot_description_xml = robot_doc.toprettyxml(indent="  ")

    # legged_controller 需要一个 URDF 文件路径，因此这里把 xacro 展开的结果写入 /tmp。
    with open(generated_urdf, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_description_xml)

    robot_description = {"robot_description": robot_description_xml}

    # 给 legged_controller/legged_cheater_controller 补充 lr_pro 对应的任务文件和 IMU 名称。
    # 注意：config/lr_pro/task.info 与 reference.info 需要后续按 lr_pro 参数单独补齐。
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

    # 发布 TF。真机控制时也需要它把 URDF 中的关节状态转换为 link TF。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control_node 会根据 robot_description 中的 <ros2_control> 加载 lr_pro_hw/LrProHW。
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml, controller_params],
    )

    # joint_state_broadcaster 用于发布 /joint_states，供 robot_state_publisher 和调试工具使用。
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 主运动控制器。它会读取 lr_pro_hw 导出的状态接口，并写入 hybrid joint command 接口。
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
            # robot_type 会传给 xacro，当前默认固定为 lr_pro。
            DeclareLaunchArgument("robot_type", default_value="lr_pro"),
            # 这两个文件属于控制器/OCS2 参数，不属于硬件接口；需要按 lr_pro 单独准备。
            DeclareLaunchArgument("task_file", default_value=os.path.join(controllers_share, "config", "lr_pro", "task.info")),
            DeclareLaunchArgument("reference_file", default_value=os.path.join(controllers_share, "config", "lr_pro", "reference.info")),
            # 硬件插件名必须和 lr_pro_hw_plugins.xml 中的 class name 一致。
            DeclareLaunchArgument("hardware_plugin", default_value="lr_pro_hw/LrProHW"),
            # 以下参数会进入 robot.xacro 的 ros2_control/gazebo 相关宏，保持和 A1 风格一致。
            DeclareLaunchArgument("power_limit", default_value="4"),
            DeclareLaunchArgument("contact_threshold", default_value="40"),
            DeclareLaunchArgument("delay", default_value="0.009"),
            OpaqueFunction(function=launch_setup),
        ]
    )
