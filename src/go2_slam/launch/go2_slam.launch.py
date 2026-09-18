"""Pipeline đầy đủ: Ouster OS1-32 -> FAST-LIO2 -> rviz2

CÂY TF SAU KHI CHẠY
-------------------
    camera_init              <- khung thế giới, FAST-LIO đặt tại vị trí khởi động
     └── body                <- ĐỘNG, FAST-LIO publish. Trùng vật lý với os_imu.
          ├── os_sensor      <- static, = inv(imu_to_sensor_transform)
          │    ├── os_lidar  <- static, driver ouster publish
          │    └── os_imu    <- static, driver ouster publish
          └── base_link      <- static, thân robot Go2 (bạn tự đo)

VÌ SAO NỐI body -> os_sensor CHỨ KHÔNG PHẢI body -> os_imu
----------------------------------------------------------
Về vật lý `body` của FAST-LIO CHÍNH LÀ os_imu. Nhưng driver ouster đã publish
os_sensor -> os_imu, mà TF2 cấm một frame có hai cha. Nối vào os_sensor bằng
phép biến đổi nghịch đảo cho ra cùng kết quả hình học mà cây vẫn hợp lệ.
"""

import os

import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ============================================================================
#  PHẦN BẠN SỬA
# ============================================================================

# --- 1. Extrinsic IMU->SENSOR của chính con lidar này ------------------------
# Sinh bằng: python3 scripts/ouster_extrinsic.py 192.168.123-metadata.json
# Giá trị dưới đây LÀ CỦA MÁY SN 122544004472. Đổi lidar thì phải chạy lại script.
BODY_TO_OS_SENSOR_XYZ = (0.002441000, 0.009725000, -0.007533000)
BODY_TO_OS_SENSOR_QUAT = (0.0, 0.0, 0.0, 1.0)          # x, y, z, w

# --- 2. Vị trí lidar trên lưng Go2 -------------------------------------------
# Tư thế của os_sensor ĐO TRONG hệ base_link của Go2.
# base_link Go2: gốc ở tâm thân, x hướng về đầu, y sang trái, z lên trên.
#
# CÁCH ĐO: đo từ tâm hình học thân chó tới tâm vỏ lidar. Mốc của os_sensor là
# mặt đáy vỏ, ngay giữa trục quay.
#
# Sai số 1-2 cm ở TỊNH TIẾN không ảnh hưởng chất lượng bản đồ (SLAM chỉ dùng
# LiDAR+IMU); nó chỉ ảnh hưởng khi chiếu bản đồ về hệ chân robot để điều hướng.
# Ngược lại GÓC XOAY phải đúng — lệch 5 độ là mặt sàn trong bản đồ bị nghiêng.
#
# GIÁ TRỊ DƯỚI ĐÂY LÀ HIỆU CHUẨN THẬT, không phải ước lượng.
# Nguồn: `lidar_calibrate` chạy trên robot 16/09/2026, rmse 1,2 cm, overlap 0,63.
# Chép từ README của bag GO2_KHUD_16-09.
# Thay cho ước lượng bằng mắt cũ (0.10, 0.0, 0.15) — lệch 14,5 cm theo x, vì
# lidar thực tế ngồi hẳn về phía trước trên vùng vai chứ không phải giữa lưng.
#
# PHẢI HIỆU CHUẨN LẠI nếu tháo lắp lidar, đổi đế, hay chỉ vặn lại ốc: đây là
# tư thế vật lý, không phải hằng số của thiết bị.
MOUNT_XYZ = (0.24525, -0.03882, 0.10411)     # mét
MOUNT_RPY = (-0.01246, 0.02703, 0.03784)     # radian: roll, pitch, yaw

# ============================================================================


def rpy_to_matrix(roll, pitch, yaw):
    """Quy ước ROS: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_to_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R):
    """Shepperd — chọn nhánh theo phần tử đường chéo lớn nhất cho ổn định số học."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def make_tf(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def stf_args(T, parent, child):
    """Ma trận 4x4 -> tham số của static_transform_publisher.

    CẢNH BÁO PHIÊN BẢN: Foxy dùng tham số VỊ TRÍ:
        x y z qx qy qz qw frame_id child_frame_id
    Từ Humble mới có dạng --x --y ... Chép nhầm cú pháp Humble vào Foxy thì node
    coi '--x' là toạ độ và ném lỗi parse.
    """
    q = matrix_to_quat(T[:3, :3])
    t = T[:3, 3]
    return [f'{t[0]:.9f}', f'{t[1]:.9f}', f'{t[2]:.9f}',
            f'{q[0]:.9f}', f'{q[1]:.9f}', f'{q[2]:.9f}', f'{q[3]:.9f}',
            parent, child]


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_slam')
    launch_dir = os.path.join(pkg_share, 'launch')

    T_body_sensor = make_tf(quat_to_matrix(*BODY_TO_OS_SENSOR_QUAT),
                            np.array(BODY_TO_OS_SENSOR_XYZ))
    T_base_sensor = make_tf(rpy_to_matrix(*MOUNT_RPY), np.array(MOUNT_XYZ))
    # body -> base_link = (body->os_sensor) @ (os_sensor->base_link)
    T_body_base = T_body_sensor @ np.linalg.inv(T_base_sensor)

    args = [
        DeclareLaunchArgument('sensor_hostname', default_value='192.168.123.40'),
        DeclareLaunchArgument('lidar_mode', default_value='1024x10'),
        DeclareLaunchArgument('timestamp_mode', default_value='TIME_FROM_ROS_TIME'),
        DeclareLaunchArgument('config_file',
                              default_value=os.path.join(pkg_share, 'config',
                                                         'ouster_os1_32.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scan_2d', default_value='false',
                              description='ép point cloud thành LaserScan 2D'),
    ]

    # ------------------------------------------------------------------
    #  VÌ SAO PHẢI BỌC TRONG GroupAction(scoped=True)
    #
    #  launch_arguments của IncludeLaunchDescription được hiện thực bằng
    #  SetLaunchConfiguration, và các biến đó RÒ RỈ RA PHẠM VI CHA sau khi
    #  include chạy xong. LaunchDescription xử lý entity theo THỨ TỰ, nên:
    #
    #      [ouster, fastlio, tf..., rviz, ...]
    #                  |                 |
    #                  |                 +-- đọc LaunchConfiguration('rviz')
    #                  +-- đặt 'rviz' = 'false' cho chính nó, nhưng rò ra ngoài
    #
    #  Kết quả: node rviz phía sau đọc phải 'false' và bị IfCondition loại bỏ,
    #  dù DeclareLaunchArgument('rviz') có default_value='true'.
    #
    #  Triệu chứng: rviz không mở, log KHÔNG có dòng nào nhắc tới rviz, không
    #  có lỗi nào cả — node đơn giản là không được tạo.
    #
    #  scoped=True tạo phạm vi riêng cho include, biến không rò ra ngoài nữa.
    # ------------------------------------------------------------------
    ouster = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ouster.launch.py')),
            launch_arguments={
                'sensor_hostname': LaunchConfiguration('sensor_hostname'),
                'lidar_mode': LaunchConfiguration('lidar_mode'),
                'timestamp_mode': LaunchConfiguration('timestamp_mode'),
            }.items()),
    ], scoped=True)

    fastlio = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'fastlio.launch.py')),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                # fastlio.launch.py tự mở rviz được (mặc định true) để dùng
                # độc lập khi phát lại rosbag. Ở đây tắt vì file này đã có rviz
                # riêng — không tắt thì mở HAI cửa sổ rviz2 cùng lúc.
                'rviz': 'false',
            }.items()),
    ], scoped=True)

    tf_sensor = Node(package='tf2_ros', executable='static_transform_publisher',
                     name='tf_body_to_os_sensor', output='log',
                     arguments=stf_args(T_body_sensor, 'body', 'os_sensor'))

    tf_base = Node(package='tf2_ros', executable='static_transform_publisher',
                   name='tf_body_to_base_link', output='log',
                   arguments=stf_args(T_body_base, 'body', 'base_link'))

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'go2_slam.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz')), output='log')

    # Bản đồ 2D cho điều hướng: cắt một lát ngang quanh tầm cao thân chó.
    scan_2d = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node', name='pc_to_scan',
        condition=IfCondition(LaunchConfiguration('scan_2d')),
        remappings=[('cloud_in', '/cloud_registered_body'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',
            # Lát cắt so với base_link: 10cm tới 60cm trên thân chó. Cắt thấp
            # hơn sẽ dính mặt đất khi robot nghiêng người lúc bước.
            'min_height': 0.10,
            'max_height': 0.60,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,   # ~0.5 độ
            'scan_time': 0.1,            # khớp 10 Hz của lidar
            'range_min': 0.3,
            'range_max': 100.0,
            'use_inf': True,
        }])

    return LaunchDescription(args + [ouster, fastlio, tf_sensor, tf_base, rviz, scan_2d])
