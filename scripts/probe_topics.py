#!/usr/bin/env python3
"""
Đo tần số topic với ĐÚNG QoS của sensor.

VÌ SAO CẦN CÔNG CỤ NÀY
----------------------
Trên ROS 2 Foxy, `ros2 topic hz` và `ros2 topic echo` luôn subscribe ở chế độ
RELIABLE và KHÔNG có cờ để đổi (cờ --qos-reliability chỉ có từ Galactic).

Driver Ouster publish point cloud ở chế độ BEST_EFFORT (SensorDataQoS) vì đó là
lựa chọn đúng cho dữ liệu tần suất cao. Hai bên lệch QoS => DDS không kết nối,
và `ros2 topic hz` treo im lặng, không báo lỗi gì.

Kết quả: bạn tưởng lidar hỏng trong khi nó đang chạy hoàn hảo.

Script này subscribe đúng bằng qos_profile_sensor_data nên đo được thật.

CÁCH DÙNG
---------
    python3 scripts/probe_topics.py                  # mặc định 6 giây
    python3 scripts/probe_topics.py --duration 15
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cloud', default='/ouster/points')
    ap.add_argument('--imu', default='/ouster/imu')
    ap.add_argument('--duration', type=float, default=6.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node('probe_topics')
    count = {'cloud': 0, 'imu': 0}
    meta = {}

    def on_cloud(msg):
        count['cloud'] += 1
        if not meta:
            meta['frame'] = msg.header.frame_id
            meta['width'] = msg.width
            meta['height'] = msg.height
            meta['point_step'] = msg.point_step
            meta['fields'] = [f.name for f in msg.fields]

    def on_imu(_):
        count['imu'] += 1

    node.create_subscription(PointCloud2, args.cloud, on_cloud, qos_profile_sensor_data)
    node.create_subscription(Imu, args.imu, on_imu, qos_profile_sensor_data)

    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    dt = time.time() - t0

    hz_cloud = count['cloud'] / dt
    hz_imu = count['imu'] / dt

    print()
    print(f"  {args.cloud:22s} {count['cloud']:5d} msg / {dt:.1f}s  ->  {hz_cloud:7.2f} Hz")
    print(f"  {args.imu:22s} {count['imu']:5d} msg / {dt:.1f}s  ->  {hz_imu:7.2f} Hz")

    if meta:
        n = meta['width'] * meta['height']
        print()
        print(f"  frame_id   : {meta['frame']}")
        print(f"  kích thước : {meta['width']} x {meta['height']} = {n} điểm/scan")
        print(f"  point_step : {meta['point_step']} byte/điểm")
        print(f"  fields     : {', '.join(meta['fields'])}")

        # FAST-LIO cần đúng bộ trường này (struct ouster_ros::Point).
        # Thiếu 't' là không khử được méo chuyển động -> bản đồ nhoè.
        need = {'x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range'}
        missing = need - set(meta['fields'])
        print()
        if missing:
            print(f"  ⚠ THIẾU trường: {sorted(missing)}")
            print(f"    -> đặt point_type:=original cho driver ouster")
        else:
            print("  ✓ Đủ trường cho FAST-LIO (point_type=original)")

    print()
    if hz_cloud < 1.0:
        print("  ⚠ Không nhận được point cloud. Kiểm tra: driver có chạy không,")
        print("    ROS_DOMAIN_ID có khớp không, CycloneDDS có chọn đúng card mạng không.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
