#!/usr/bin/env python3
"""
Phát /joint_states toàn số 0 cho 12 khớp chân của Go2.

VÌ SAO KHÔNG DÙNG joint_state_publisher
---------------------------------------
Gói đó đọc URDF từ TOPIC /robot_description, mà trên CycloneDDS 0.7 (bản của
Foxy) topic durability TRANSIENT_LOCAL này không tới được subscriber nào —
`ros2 topic echo /robot_description` cũng trống, dù publisher tồn tại và khai
đúng TRANSIENT_LOCAL. Nên joint_state_publisher treo ở
"Waiting for robot_description to be published..." vĩnh viễn.

Node này đọc thẳng URDF từ FILE nên không dính lỗi đó.

robot_state_publisher không gặp vấn đề vì nó nhận URDF qua THAM SỐ, không
qua topic.

TƯ THẾ CHÂN
-----------
Tất cả khớp bằng 0 nghĩa là con chó đứng ở tư thế cố định, không cử động theo
robot thật. Muốn chân đúng thật thì phải cầu nối /lowstate của Unitree (chạy
trên DDS riêng của họ) sang sensor_msgs/JointState. Với mục đích xem SLAM thì
không cần: cái quan trọng là vị trí và hướng của THÂN robot trong bản đồ.
"""

import argparse
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def movable_joints(urdf_path):
    """Lấy tên các khớp KHÔNG cố định — chỉ những khớp này cần joint_states."""
    root = ET.parse(urdf_path).getroot()
    return [j.get('name') for j in root.findall('joint')
            if j.get('type') not in ('fixed', None)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf',
                    default='/ws/install/go2_description/share/go2_description/urdf/go2_description.urdf')
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    names = movable_joints(args.urdf)

    rclpy.init()
    node = Node('joint_zeros')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    node.get_logger().info(f'phát {len(names)} khớp ở {args.rate} Hz: {", ".join(names)}')

    msg = JointState()
    msg.name = names
    msg.position = [0.0] * len(names)

    def tick():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    node.create_timer(1.0 / args.rate, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
