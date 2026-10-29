"""F2 流程：手动转视觉降落（需求书 §5.6 F2）。

    ros2 launch drone_bringup flow_f2.launch.py

链路：sim_bringup（PX4 + gz + 桥接 + mavros）
      → aruco_detector（/aruco/detection）
      → landing_controller（状态机，/aruco/landing/state）
      → mission_manager（模式调度，按 '2' 切 ARUCO_LAND 时把控制权交给 landing_controller）

人工试飞：加 start_teleop:=true，起飞后飞到码附近，按 '2' 后就不用再管。
自动验收：scripts/dev_run_f2.sh 会用 verify_flow 直接发同样的模式请求。
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

    # 检测节点单独一个 launch 文件，参数一律以 aruco.yaml 为准（不要在这里再给 marker_size，
    # M1 踩过：launch 默认值会覆盖 yaml 里标定好的 0.487）
    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'aruco_detect.launch.py')),
        condition=IfCondition(start_detector),
        launch_arguments={'use_sim_time': use_sim_time}.items())

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
                'flow': 'F2',
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
        declare_args + [sim, detector, landing, mission_manager, teleop])
