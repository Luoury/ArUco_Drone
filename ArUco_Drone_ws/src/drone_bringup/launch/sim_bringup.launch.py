"""SITL 仿真桥接 + MAVROS 启动（M0 起所有流程共用的底座）。

启动顺序（需求书 §5.1）：
  1) scripts/sim.sh start                      # PX4 + Gazebo（headless + Ogre1）
  2) ros2 launch drone_bringup sim_bringup.launch.py

桥接约定（需求书 §3.1）：
  /clock              <- gz /clock
  /camera/image_raw   <- gz .../sensor/camera/image
  /camera/camera_info <- gz .../sensor/camera/camera_info

桥接由本包的 gz_sensor_bridge 完成，不用 ros_gz_bridge：
apt 的 ros-humble-ros-gz-bridge 链接 Fortress 的 ignition-transport11，
与本机 Gazebo 8.10 的 gz-transport13 不是同一个发现域，订阅永远收不到
数据（gz topic -i 显示 No subscribers）。详见 docs/troubleshooting.md。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('drone_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    fcu_url = LaunchConfiguration('fcu_url')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    start_gz_bridge = LaunchConfiguration('start_gz_bridge')
    start_mavros = LaunchConfiguration('start_mavros')
    start_camera = LaunchConfiguration('start_camera')
    start_odom = LaunchConfiguration('start_odom')
    start_overhead = LaunchConfiguration('start_overhead')
    overhead_model = LaunchConfiguration('overhead_model')

    gz_camera_head = ["'/world/' + '", world, "' + '/model/' + '", model,
                      "' + '/link/camera_link/sensor/camera'"]
    gz_image_topic = PythonExpression(gz_camera_head + [" + '/image'"])
    gz_info_topic = PythonExpression(gz_camera_head + [" + '/camera_info'"])
    gz_odom_topic = PythonExpression(
        ["'/model/' + '", model, "' + '/odometry_with_covariance'"])
    # 俯视机位：模型名固定为 overhead_cam，挂在 /world/<world>/model/overhead_cam/...
    gz_overhead_head = ["'/world/' + '", world, "' + '/model/' + '", overhead_model,
                        "' + '/link/link/sensor/camera'"]
    gz_overhead_image_topic = PythonExpression(gz_overhead_head + [" + '/image'"])
    gz_overhead_info_topic = PythonExpression(gz_overhead_head + [" + '/camera_info'"])

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='使用 Gazebo 仿真时钟（必须桥接 /clock）'),
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14580',
                              description='MAVROS 连接串（需求书 §2：bind 14540 → 14580）'),
        DeclareLaunchArgument('world', default_value='aruco', description='gz 世界名'),
        DeclareLaunchArgument('model', default_value='x500_mono_cam_down_0',
                              description='gz 模型实例名（<sim_model 去掉 gz_>_<instance>）'),
        DeclareLaunchArgument('start_gz_bridge', default_value='true',
                              description='是否桥接 /clock'),
        DeclareLaunchArgument('start_mavros', default_value='true',
                              description='是否启动 MAVROS'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='是否桥接相机图像（排查时可先关掉）'),
        DeclareLaunchArgument('start_odom', default_value='true',
                              description='是否桥接机体真值位姿到 /gz/drone_pose（精度验证用）'),
        DeclareLaunchArgument('start_overhead', default_value='false',
                              description='是否桥接俯视机位（录视频用；需先 spawn overhead_cam）'),
        DeclareLaunchArgument('overhead_model', default_value='overhead_cam',
                              description='俯视机位的 gz 模型名'),
    ]

    gz_sensor_bridge = Node(
        package='drone_bringup',
        executable='gz_sensor_bridge',
        name='gz_sensor_bridge',
        output='screen',
        parameters=[
            PathJoinSubstitution([bringup_share, 'config', 'bridge.yaml']),
            {
                'enable_clock': start_gz_bridge,
                'enable_camera': start_camera,
                'enable_odom': start_odom,
                'gz_clock_topic': '/clock',
                'gz_image_topic': gz_image_topic,
                'gz_info_topic': gz_info_topic,
                'gz_odom_topic': gz_odom_topic,
                'frame_id': 'camera_link',
                'enable_overhead': start_overhead,
                'gz_overhead_image_topic': gz_overhead_image_topic,
                'gz_overhead_info_topic': gz_overhead_info_topic,
            },
        ],
    )

    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([bringup_share, 'config', 'mavros.yaml']),
            {'fcu_url': fcu_url, 'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(start_mavros),
    )

    return LaunchDescription(declared + [gz_sensor_bridge, mavros])
