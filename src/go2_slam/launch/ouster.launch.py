"""Driver Ouster OS-1-32-U2-SR.

Khai báo Node trực tiếp thay vì include sensor.launch.xml của ouster_ros để
thấy rõ từng tham số và không kéo theo rviz mặc định của họ.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # IP của lidar. Máy này đang ở 192.168.123.40 (cùng dải với Go2).
        DeclareLaunchArgument('sensor_hostname', default_value='192.168.123.40'),

        # Để trống = driver tự suy ra IP máy nhận từ bảng định tuyến.
        DeclareLaunchArgument('udp_dest', default_value=''),

        # 1024x10 = 1024 cột, 10 vòng/giây. Cân bằng tốt nhất cho robot đi bộ.
        # 2048x10 gấp đôi dữ liệu, lợi ích không đáng kể.
        # 1024x20 cho mỗi scan quá thưa với máy 32 tia -> dễ mất bám.
        DeclareLaunchArgument('lidar_mode', default_value='1024x10'),

        # TIME_FROM_ROS_TIME : lấy thời điểm nhận gói đầu mỗi scan. Khớp đồng hồ
        #     ROS ngay, thêm jitter dưới 1 ms. Dùng cái này để chạy được liền.
        # TIME_FROM_INTERNAL_OSC : đồng hồ riêng của sensor, lệch hàng tỉ giây so
        #     với ROS -> rviz báo "message too old", không ghép được với Go2.
        # TIME_FROM_PTP_1588 : chuẩn nhất, cần chạy linuxptp trên máy chủ.
        DeclareLaunchArgument('timestamp_mode', default_value='TIME_FROM_ROS_TIME'),

        DeclareLaunchArgument('metadata', default_value=''),
    ]

    driver = Node(
        package='ouster_ros', executable='os_driver',
        name='os_driver', namespace='ouster', output='screen',
        parameters=[{
            'sensor_hostname': LaunchConfiguration('sensor_hostname'),
            'udp_dest':        LaunchConfiguration('udp_dest'),
            'lidar_mode':      LaunchConfiguration('lidar_mode'),
            'timestamp_mode':  LaunchConfiguration('timestamp_mode'),
            'metadata':        LaunchConfiguration('metadata'),

            # 0 = tự chọn port trống (mặc định sensor gửi tới 7502/7503)
            'lidar_port': 0,
            'imu_port': 0,

            # Tên frame — phải khớp static TF trong go2_slam.launch.py
            'sensor_frame': 'os_sensor',
            'lidar_frame':  'os_lidar',
            'imu_frame':    'os_imu',

            # Point cloud trong hệ os_lidar. Ở chế độ này driver KHÔNG áp
            # lidar_to_sensor_transform, nên extrinsic cho FAST-LIO phải là
            # T_imu_lidar — đúng cái ouster_extrinsic.py tính với --frame lidar.
            'point_cloud_frame': 'os_lidar',

            # IMG = ảnh range/signal/nearir (không cần cho SLAM, tắt để đỡ CPU)
            # SCAN = LaserScan 2D (tắt, ta tự tạo bằng pointcloud_to_laserscan)
            'proc_mask': 'PCL|IMU',

            # 'original' -> struct ouster_ros::Point đủ 9 trường
            # (x,y,z,intensity,t,reflectivity,ring,ambient,range).
            # Đổi sang xyzir/native là FAST-LIO mất trường 't' -> không khử được
            # méo chuyển động -> bản đồ nhoè.
            'point_type': 'original',

            # false = SensorDataQoS (best-effort), đúng cho dữ liệu tần suất cao.
            # LƯU Ý: 'ros2 topic hz' của Foxy không đọc được best-effort và sẽ
            # im lặng. Dùng scripts/probe_topics.py để đo.
            'use_system_default_qos': False,

            # false bỏ điểm NaN, giảm ~20% băng thông. FAST-LIO không cần lưới.
            'organized': False,
            'destagger': True,

            'min_range': 0.3,
            'max_range': 120.0,

            'persist_config': False,
            'auto_start': True,
        }],
    )

    return LaunchDescription(args + [driver])
