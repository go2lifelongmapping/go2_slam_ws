"""Hiện mô hình 3D của Go2 trong rviz.

Chạy kèm go2_slam.launch.py hoặc fastlio.launch.py:
    ros2 launch go2_slam robot_model.launch.py

VÌ SAO GHÉP ĐƯỢC NGAY
---------------------
Link gốc của URDF go2_description là `base_link`, trùng đúng frame mà
go2_slam.launch.py đã phát static transform tới (`body` -> `base_link`).
Cây TF vì thế nối liền:

    camera_init --(FAST-LIO odometry)--> body --(static)--> base_link
                                                              |
                                              robot_state_publisher
                                                              v
                                            29 link của con chó (hip/thigh/calf/foot)

KHỚP CHÂN ĐỨNG YÊN
------------------
scripts/joint_zeros.py phát giá trị 0 cho cả 12 khớp chân, nên con chó hiện ra
ở tư thế cố định chứ không cử động theo robot thật. Muốn chân nhúc nhích đúng
thì phải cầu nối `/lowstate` của Unitree sang sensor_msgs/JointState — Unitree
phát nó trên DDS riêng của họ, không phải topic ROS trong domain này.

Với mục đích xem SLAM thì tư thế chân không quan trọng: cái cần là biết thân
robot đang ở đâu và quay hướng nào trong bản đồ.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_go2_slam_launch():
    """Nạp go2_slam.launch.py như một module để dùng CHUNG hằng số extrinsic.

    Không chép lại MOUNT_XYZ/MOUNT_RPY sang đây: chép là sớm muộn hai bản lệch
    nhau, và mô hình robot sẽ lệch khỏi đám mây điểm mà không ai biết vì sao.
    """
    pkg = get_package_share_directory('go2_slam')
    path = os.path.join(pkg, 'launch', 'go2_slam.launch.py')
    spec = importlib.util.spec_from_file_location('go2_slam_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('go2_description'),
                        'urdf', 'go2_description.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    g = _load_go2_slam_launch()
    T_body_sensor = g.make_tf(g.quat_to_matrix(*g.BODY_TO_OS_SENSOR_QUAT),
                              g.np.array(g.BODY_TO_OS_SENSOR_XYZ))
    T_base_sensor = g.make_tf(g.rpy_to_matrix(*g.MOUNT_RPY), g.np.array(g.MOUNT_XYZ))
    T_body_base = T_body_sensor @ g.np.linalg.inv(T_base_sensor)

    return LaunchDescription([
        # body -> base_link, CHỈ bật khi chạy kèm fastlio.launch.py (phát lại
        # bag). Chạy kèm go2_slam.launch.py thì file đó đã phát rồi — bật cả
        # hai là hai node cùng phát một transform, TF sẽ chập chờn.
        DeclareLaunchArgument('publish_base_tf', default_value='false'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_body_to_base_link_rm', output='log',
             arguments=g.stf_args(T_body_base, 'body', 'base_link'),
             condition=IfCondition(LaunchConfiguration('publish_base_tf'))),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='log',
            # Foxy nhận URDF dưới dạng CHUỖI trong tham số robot_description,
            # không phải đường dẫn file.
            parameters=[{'robot_description': robot_desc}],
        ),

        # KHÔNG dùng joint_state_publisher: nó đọc URDF từ topic
        # /robot_description, mà topic TRANSIENT_LOCAL đó không tới được
        # subscriber nào trên CycloneDDS 0.7 của Foxy (ros2 topic echo cũng
        # trống). Script dưới đọc thẳng từ file nên không dính lỗi ấy.
        ExecuteProcess(
            cmd=['python3', '/ws/scripts/joint_zeros.py', '--urdf', urdf],
            output='log',
        ),
    ])
