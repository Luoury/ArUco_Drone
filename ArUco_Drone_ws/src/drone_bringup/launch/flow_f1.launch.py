"""F1 流程：纯手动 —— 起飞 → 键盘飞一圈 → 降落（需求书 §7 F1）。

    ros2 launch drone_bringup flow_f1.launch.py

默认不起 teleop（无 TTY 的自动化场景会直接失败），人工试飞时显式打开：

    ros2 launch drone_bringup flow_f1.launch.py start_teleop:=true
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_teleop = LaunchConfiguration('start_teleop')
    world = LaunchConfiguration('world')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='aruco'),
        DeclareLaunchArgument(
            'start_teleop', default_value='false',
            description='是否同时启动键盘遥控（需要真实 TTY）'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
        }.items())

    mission_manager = Node(
        package='drone_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            flows_yaml,
            {'use_sim_time': use_sim_time, 'flow': 'F1'},
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

    return LaunchDescription(declare_args + [sim, mission_manager, teleop])
