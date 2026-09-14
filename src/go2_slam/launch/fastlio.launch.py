"""Chỉ chạy FAST-LIO2, không có driver.

Tách riêng để: chạy lại SLAM trên rosbag đã ghi mà không cần lidar, và restart
SLAM khi chỉnh tham số mà không phải reset sensor.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, 'config', 'ouster_os1_32.yaml')),

        # LƯU Ý: 'ros2 bag play' của Foxy KHÔNG có cờ --clock (chỉ có từ
        # Galactic). Nên khi phát lại bag, cứ để false. FAST-LIO vẫn chạy đúng
        # vì nó so dấu thời gian LiDAR với IMU bên trong dữ liệu, cả hai đều
        # từ bag nên nhất quán với nhau.
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument('rviz', default_value='true',
                              description='mở rviz2 kèm theo'),

        Node(
            package='fast_lio', executable='fastlio_mapping',
            name='fastlio_mapping', output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'go2_slam.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
