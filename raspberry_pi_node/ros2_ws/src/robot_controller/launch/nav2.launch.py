"""
nav2.launch.py
==============
ROS 2 Nav2 통합 런치 파일

시작되는 노드:
  1. lidar_node          — RPLIDAR A1 스캔 (/scan, /forward_distance)
  2. motor_node          — /cmd_vel → CAN TX
  3. keyboard_node       — MANUAL 키보드 제어 + /mode 토글
  4. nav2_goal_publisher — AUTO 모드 전방 목표 게시
  5. rf2o_laser_odometry — LiDAR 스캔 매칭 오도메트리 (/odom)
  6. static_transform    — base_link → laser_link (정적 TF)
  7. Nav2 stack          — BT, Controller, Planner, Costmaps, Lifecycle Manager

전제 조건:
  sudo ip link set can0 up type can bitrate 500000
  pip install -r requirements.txt
  sudo apt install ros-jazzy-nav2-bringup ros-jazzy-rf2o-laser-odometry

사용법:
  ros2 launch robot_controller nav2.launch.py
  ros2 launch robot_controller nav2.launch.py goal_distance_m:=5.0
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_controller')

    # ── Launch 인수 ───────────────────────────────────────────────
    goal_dist_arg = DeclareLaunchArgument(
        'goal_distance_m', default_value='3.0',
        description='AUTO 모드 전방 목표 거리 (m)'
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Nav2 스택 활성화 여부'
    )
    fov_arg = DeclareLaunchArgument(
        'fov_degrees', default_value='60.0',
        description='전방 감시 시야각 (도)'
    )

    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # ── 1. LiDAR 노드 ─────────────────────────────────────────────
    lidar_node = Node(
        package='robot_controller',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[{
            'port':         '/dev/ttyUSB0',
            'baud':         115200,
            'fov_degrees':  LaunchConfiguration('fov_degrees'),
            'lidar_offset': 0.0,
        }],
    )

    # ── 2. 모터 노드 ──────────────────────────────────────────────
    motor_node = Node(
        package='robot_controller',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[{
            'can_channel': 'can0',
            'can_id':      0x123,
            'max_speed':   9999,
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),   # Nav2 출력 → 모터 입력
        ],
    )

    # ── 3. 키보드 노드 (별도 터미널) ─────────────────────────────
    keyboard_node = Node(
        package='robot_controller',
        executable='keyboard_node',
        name='keyboard_node',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'normal_speed': 0.2002,
            'boost_speed':  0.4001,
        }],
    )

    # ── 4. Nav2 목표 게시 노드 ────────────────────────────────────
    nav2_goal_publisher = Node(
        package='robot_controller',
        executable='nav2_goal_publisher',
        name='nav2_goal_publisher',
        output='screen',
        parameters=[{
            'goal_distance_m': LaunchConfiguration('goal_distance_m'),
            'update_rate_hz':  0.5,
        }],
    )

    # ── 5. rf2o_laser_odometry (엔코더 없는 스캔 매칭 오도메트리) ─
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic':  '/scan',
            'odom_topic':        '/odom',
            'publish_tf':        True,
            'base_frame_id':     'base_link',
            'odom_frame_id':     'odom',
            'init_pose_from_topic': '',
            'freq':              10.0,
        }],
    )

    # ── 6. 정적 TF: base_link → laser_link (LiDAR = base 위치) ───
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '0', '0', '0.1',   # x y z (LiDAR 높이 약 10cm)
            '0', '0', '0', '1', # qx qy qz qw (회전 없음)
            'base_link',
            'laser_link',
        ],
        output='screen',
    )

    # ── 7. Nav2 스택 (IncludeLaunchDescription) ───────────────────
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time':   'false',
            'params_file':    nav2_params_file,
            'autostart':      'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    # ── /cmd_vel 리맵 (Nav2 → motor_node) ────────────────────────
    # Nav2는 /cmd_vel 출력, motor_node는 /cmd_vel_nav 수신 후 동일하게 처리
    # 간단히 동일 토픽으로 연결:
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/cmd_vel_nav'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    return LaunchDescription([
        # Launch 인수
        goal_dist_arg,
        use_nav2_arg,
        fov_arg,

        # 로봇 노드들
        lidar_node,
        motor_node,
        keyboard_node,
        nav2_goal_publisher,

        # 오도메트리 + TF
        rf2o_node,
        static_tf_node,

        # Nav2 스택
        nav2_launch,
        cmd_vel_relay,
    ])
