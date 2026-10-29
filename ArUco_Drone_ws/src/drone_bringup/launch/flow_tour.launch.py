"""多工位巡回演示（TOUR）：一条命令跑完"基站 → 三个工位 → 回基站"。

与其它 flow_F*.launch.py 的区别：
- 检测端用 marker_id:=-1（任意码，取画面中心最近的那个），因为每个工位有自己的码；
- mission_manager 额外装 config/tour.yaml（工位列表与超时），并把 flow 设为 TOUR；
- 世界里的三个额外码与演示机位由 scripts/spawn_course.sh 摆放（不写死在 world 文件里）。

用法：
    ros2 launch drone_bringup flow_tour.launch.py start_overhead:=true overhead_model:=demo_cam
    # 起来后发一次模式请求：
    ros2 topic pub --once /mission/mode std_msgs/msg/String "{data: TOUR}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('drone_bringup')
    launch_dir = os.path.join(bringup_share, 'launch')
    flows_yaml = os.path.join(bringup_share, 'config', 'flows.yaml')
    tour_yaml = os.path.join(bringup_share, 'config', 'tour.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='aruco'),
        DeclareLaunchArgument('start_teleop', default_value='false'),
        DeclareLaunchArgument('start_detector', default_value='true'),
        DeclareLaunchArgument('start_overhead', default_value='false'),
        DeclareLaunchArgument('overhead_model', default_value='demo_cam'),
        DeclareLaunchArgument('marker_id', default_value='-1'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'start_overhead': LaunchConfiguration('start_overhead'),
            'overhead_model': LaunchConfiguration('overhead_model'),
        }.items(),
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'aruco_detect.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_detector')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 任意码：每个工位一个 id，检测端取画面中心最近的那个
            'marker_id': LaunchConfiguration('marker_id'),
        }.items(),
    )

    landing = Node(
        package='drone_aruco',
        executable='landing_controller',
        name='landing_controller',
        output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'landing.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    mission = Node(
        package='drone_mission',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            flows_yaml,
            tour_yaml,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'flow': 'TOUR',
            },
        ],
    )

    teleop = Node(
        package='drone_teleop',
        executable='teleop_node',
        name='drone_teleop',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_teleop')),
        parameters=[
            flows_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription(args + [sim, detector, landing, mission, teleop])
