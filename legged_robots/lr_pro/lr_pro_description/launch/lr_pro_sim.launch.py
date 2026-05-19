import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# lr_pro Gazebo 仿真控制启动入口。
# 和 display.launch.py 的区别：这里会加载 gazebo_ros2_control，并自动启动 legged_controller。
def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    task_file = LaunchConfiguration("task_file").perform(context)
    reference_file = LaunchConfiguration("reference_file").perform(context)
    power_limit = LaunchConfiguration("power_limit").perform(context)
    contact_threshold = LaunchConfiguration("contact_threshold").perform(context)
    delay = LaunchConfiguration("delay").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)

    description_share = get_package_share_directory("lr_pro_description")
    gazebo_share = get_package_share_directory("legged_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    robot_xacro = os.path.join(description_share, "robot.xacro")
    world_file = os.path.join(gazebo_share, "worlds", "empty_world.world")
    generated_urdf = os.path.join("/tmp", f"lr_pro_{robot_type}_gazebo.urdf")
    controller_params_file = os.path.join("/tmp", f"lr_pro_{robot_type}_gazebo_controllers.yaml")

    # Gazebo 中的 controller_manager 由 gazebo_ros2_control 插件创建。
    # 这里生成第二份临时参数文件，用于给 legged_controller 指定 lr_pro 专属 task/reference/urdf。
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

    # 仿真硬件插件固定使用 legged_gazebo/LeggedHWSim；真实机器人不要用这个 launch。
    robot_doc = xacro.process_file(
        robot_xacro,
        mappings={
            "robot_type": robot_type,
            "hardware_plugin": "legged_gazebo/LeggedHWSim",
            "power_limit": power_limit,
            "contact_threshold": contact_threshold,
            "delay": delay,
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
            "pause": "true",
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
        arguments=["-topic", "robot_description", "-entity", robot_type, "-z", spawn_z],
        output="screen",
    )

    # Humble 的 spawn_entity.py 不支持 -J，因此用 Gazebo 服务设置模型初始关节角。
    # Gazebo 以 pause=true 启动，避免设置关节角之前模型从零姿态自由下落。
    initial_joint_request = {
        "model_name": robot_type,
        "urdf_param_name": "robot_description",
        "joint_names": [
            "LF_HAA", "LF_HFE", "LF_KFE",
            "RF_HAA", "RF_HFE", "RF_KFE",
            "LH_HAA", "LH_HFE", "LH_KFE",
            "RH_HAA", "RH_HFE", "RH_KFE",
        ],
        "joint_positions": [
            0.00, 0.82, -1.64,
            0.00, 0.82, -1.64,
            0.00, 0.82, -1.64,
            0.00, 0.82, -1.64,
        ],
    }
    set_initial_configuration = ExecuteProcess(
        cmd=[
            "timeout",
            "8s",
            "ros2",
            "service",
            "call",
            "/set_model_configuration",
            "gazebo_msgs/srv/SetModelConfiguration",
            yaml.safe_dump(initial_joint_request, default_flow_style=True),
        ],
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

    unpause_physics = ExecuteProcess(
        cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
    )

    # 顺序：spawn -> 等 Gazebo 注册模型 -> 设置初始关节角 -> 加载 controller -> 放开物理仿真。
    set_initial_configuration_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=1.0, actions=[set_initial_configuration])],
        )
    )
    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=set_initial_configuration,
            on_exit=[joint_state_spawner, legged_controller_spawner],
        )
    )
    unpause_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=legged_controller_spawner,
            on_exit=[unpause_physics],
        )
    )

    return [
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        set_initial_configuration_after_spawn,
        spawn_controllers,
        unpause_after_controller,
    ]


def generate_launch_description():
    controllers_share = get_package_share_directory("legged_controllers")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="lr_pro"),
            DeclareLaunchArgument("task_file", default_value=os.path.join(controllers_share, "config", "lr_pro", "task.info")),
            DeclareLaunchArgument("reference_file", default_value=os.path.join(controllers_share, "config", "lr_pro", "reference.info")),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("spawn_z", default_value="0.53"),
            DeclareLaunchArgument("power_limit", default_value="4"),
            DeclareLaunchArgument("contact_threshold", default_value="40"),
            DeclareLaunchArgument("delay", default_value="0.009"),
            OpaqueFunction(function=launch_setup),
        ]
    )
