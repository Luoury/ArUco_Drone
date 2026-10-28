"""只启动 ArUco 检测节点。

前提：sim_bringup.launch.py 已经在跑，提供 /camera/image_raw、/camera/camera_info
以及机体位姿。字典、ID、marker_size 等标定值一律由 config/aruco.yaml 提供，
这里的 launch 参数只覆盖"运行期需要切换的开关"，避免两处默认值互相打架
（曾经踩过：launch 里 marker_size 默认 0.50 把 yaml 里标定好的 0.487 覆盖掉了）。

drone_pose_topic 默认用 mavros 的估计位姿；做精度验证时改成 /gz/drone_pose
（gz 世界系真值），这样检测误差里就不含 PX4 EKF 的估计误差。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('drone_bringup')
    aruco_yaml = os.path.join(bringup_share, 'config', 'aruco.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_debug', default_value='true'),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('drone_pose_topic',
                              default_value='/mavros/local_position/pose'),
    ]

    detector = Node(
        package='drone_aruco',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[
            aruco_yaml,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_debug': LaunchConfiguration('publish_debug'),
                'marker_id': LaunchConfiguration('marker_id'),
                'drone_pose_topic': LaunchConfiguration('drone_pose_topic'),
            },
        ],
    )

    return LaunchDescription(args + [detector])
