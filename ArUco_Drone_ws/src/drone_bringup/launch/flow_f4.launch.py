"""F4 流程：全自动任务链路（需求书 §5.6 F4 / M5）。

    ros2 launch drone_bringup flow_f4.launch.py

链路：sim_bringup（PX4 + gz + 桥接 + mavros）
      → aruco_detect（/aruco/detection、/aruco/debug_image）
      → waypoint_nav（按 waypoints.yaml 飞，终点就是码正上方）
      → landing_controller（搜码 → 对准 → 下降 → 落地）
      → mission_manager（唯一调度方：收到 AUTO 后先委托航点，再委托视觉降落）

全程零人工输入：一条 AUTO 指令 → 起飞 → 3 个航点 → 回码上方 → 搜码 → 降落上锁。
自动验收：scripts/dev_run_f4.sh 用 verify_flow --flow F4 发同一条 AUTO 指令。
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
    landing_yaml = os.path.join(bringup_share, 'config', 'landing.yaml')

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
            description='是否同时启动键盘遥控（需要真实 TTY；验收时保持 false 才是"零人工输入"）'),
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

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'aruco_detect.launch.py')),
        condition=IfCondition(start_detector),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'drone_pose_topic': '/mavros/local_position/pose',
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

    landing = Node(
        package='drone_aruco',
        executable='landing_controller',
        name='landing_controller',
        output='screen',
        parameters=[
            landing_yaml,
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
                'flow': 'F4',
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
        declare_args + [sim, detector, waypoint, landing, mission_manager, teleop])
