#!/usr/bin/env python3
"""
Đọc JSON từ stdin (do lowstate_reader.py sinh ra), phát sensor_msgs/JointState.

CHẠY VỚI ROS ĐÃ SOURCE. Xem lowstate_reader.py để biết vì sao phải tách hai
tiến trình.

CÁCH DÙNG
    python3 lowstate_reader.py eth0 50 | python3 joint_state_pub.py
"""

import json
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = Node('joint_state_pub')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    log = node.get_logger()

    names, n = None, 0
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            if 'names' in d:
                names = d['names']
                log.info(f'{len(names)} khớp: {", ".join(names)}')
                continue
            if names is None:
                continue
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = names
            msg.position = d['q']
            msg.velocity = d.get('dq', [])
            pub.publish(msg)
            n += 1
            if n % 250 == 0:
                log.info(f'đã phát {n} message')
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
