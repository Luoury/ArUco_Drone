"""F3 流程：纯自主航点飞行（需求书 §5.6 F3）。

    ros2 launch drone_bringup flow_f3.launch.py

链路：sim_bringup（PX4 + gz + 桥接 + mavros）
      → waypoint_nav（按 waypoints.yaml 顺序飞行，/waypoint/state、/waypoint/progress）
      → mission_manager（模式调度，收到 WAYPOINT 时把设定点所有权交给 waypoint_nav）

全程零人工输入：起飞 → 3 个航点 → 回到码上方悬停 → 由 mission_manager 收回控制权。
自动验收：scripts/dev_run_f3.sh 用 verify_flow 发同样的模式请求。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')

    flows_yaml = os.path.join(bringup_share, 'config', 'flows.yaml')
    waypoints_yaml = os.path.join(bringup_share, 'config', 'waypoints.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_teleop = LaunchConfiguration('start_teleop')
    start_detector = LaunchConfiguration('start_detector')
    world = LaunchConfiguration('world')
    start_overhead = LaunchConfiguration('start_overhead')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='aruco'),
        DeclareLaunchArgument(
            'start_overhead', default_value='false',
            description='是否桥接俯视机位（录视频用；需先 spawn overhead_cam）'),
        DeclareLaunchArgument(
            'start_teleop', default_value='false',
            description='是否同时启动键盘遥控（需要真实 TTY）'),
        DeclareLaunchArgument(
            'start_detector', default_value='true',
            description='是否启动 ArUco 检测（调试时可关掉）'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'start_overhead': start_overhead,
        }.items())

    waypoint = Node(
        package='drone_nav',
        executable='waypoint_nav',
        name='waypoint_nav',
        output='screen',
        parameters=[
            waypoints_yaml,
            {'use_sim_time': use_sim_time},
        ])

    mission_manager = Node(
        package='drone_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            flows_yaml,
            {
                'use_sim_time': use_sim_time,
                'flow': 'F3',
            },
        ])

    teleop = Node(
        package='drone_teleop',
        executable='teleop_node',
        name='drone_teleop',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_teleop),
        parameters=[
            flows_yaml,
            {'use_sim_time': use_sim_time},
        ])

    return LaunchDescription(
        declare_args + [sim, waypoint, mission_manager, teleop])
