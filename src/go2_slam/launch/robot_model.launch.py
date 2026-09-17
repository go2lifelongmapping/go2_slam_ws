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
12 khớp chân được đổi thành `fixed` trước khi nạp (xem _freeze_joints), nên
toàn bộ TF của con chó là static và không cần /joint_states.

Chân vì thế đứng im ở tư thế góc 0. Muốn chân nhúc nhích đúng thì phải cầu nối
`/lowstate` của Unitree sang sensor_msgs/JointState — Unitree phát nó trên DDS
riêng của họ, không phải topic ROS trong domain này.

Với mục đích xem SLAM thì tư thế chân không quan trọng: cái cần là biết thân
robot đang ở đâu và quay hướng nào trong bản đồ.
"""

import importlib.util
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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


def _freeze_joints(urdf_path):
    """Đổi mọi khớp revolute thành fixed, trả về URDF dạng chuỗi.

    VÌ SAO: khớp revolute buộc robot_state_publisher phát TF lên /tf ở tần số
    cao, và khi DDS phải tải point cloud thì những TF đó BỊ BỎ ĐÓI — thân robot
    vẫn hiện (vì body->base_link là static, latched) nhưng bốn chân biến mất.

    Khớp fixed thì robot_state_publisher đẩy sang /tf_static: latched, gửi một
    lần, không bao giờ mất. Và cũng không cần /joint_states nữa.

    Đánh đổi: chân đứng im ở tư thế góc 0. Vốn dĩ đã thế rồi — chân chỉ cử động
    được nếu có nguồn joint_states thật từ robot, mà Unitree phát /lowstate trên
    DDS riêng của họ chứ không phải topic ROS trong domain này.
    """
    tree = ET.parse(urdf_path)
    n = 0
    for j in tree.getroot().findall('joint'):
        if j.get('type') in ('revolute', 'continuous', 'prismatic'):
            j.set('type', 'fixed')
            n += 1
    return ET.tostring(tree.getroot(), encoding='unicode')


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('go2_description'),
                        'urdf', 'go2_description.urdf')
    robot_desc = _freeze_joints(urdf)

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

    ])
