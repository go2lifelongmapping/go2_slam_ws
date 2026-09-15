#!/usr/bin/env python3
"""
Kiểm tra dữ liệu đầu vào của khâu khử méo chuyển động (deskew) trong FAST-LIO.

FAST-LIO khử méo bằng cách dùng trường 't' của TỪNG ĐIỂM (thời gian lệch so với
đầu scan) cùng tích phân IMU, để dịch điểm về vị trí tại thời điểm cuối scan.

Ba thứ phải đúng, script này kiểm tra cả ba:

  1. ĐƠN VỊ của 't'  -> tham số preprocess.timestamp_unit
     Với 1024x10 (100 ms mỗi scan), t phải chạy từ 0 tới ~1e8 nếu là NANO giây.
     Nếu chỉ tới ~1e5 thì là MICRO giây -> phải đặt timestamp_unit: 2.
     Sai đơn vị 1000 lần: đứng yên vẫn đẹp, di chuyển là nhoè ngay.

  2. ĐỘ ỔN ĐỊNH dấu thời gian IMU
     TIME_FROM_ROS_TIME lấy thời điểm NHẬN gói làm mốc -> có jitter mạng.
     Jitter lớn làm tích phân IMU sai khoảng -> khử méo lệch.

  3. KHOẢNG TRỐNG giữa các scan
     Phải đều 100 ms. Mất scan làm IMU phải tích phân qua khoảng dài gấp đôi.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


def main():
    rclpy.init()
    node = Node('check_deskew')
    clouds, imu_stamps = [], []

    def on_cloud(msg):
        if len(clouds) < 12:
            clouds.append(msg)

    def on_imu(msg):
        imu_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    node.create_subscription(PointCloud2, '/ouster/points', on_cloud, qos_profile_sensor_data)
    node.create_subscription(Imu, '/ouster/imu', on_imu, qos_profile_sensor_data)

    t0 = time.time()
    while time.time() - t0 < 8.0 and len(clouds) < 12:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not clouds:
        print("Không nhận được point cloud. Driver có chạy không?")
        return

    m = clouds[0]
    off = {f.name: (f.offset, f.datatype) for f in m.fields}
    print(f"point_step = {m.point_step} byte, {m.width}x{m.height} điểm")
    print(f"offset các trường: " + ", ".join(f"{k}@{v[0]}" for k, v in off.items()))
    print()

    # --- 1. Đơn vị của trường 't' ------------------------------------------
    if 't' not in off:
        print("KHÔNG có trường 't' -> FAST-LIO không khử méo được.")
        print("   Đặt point_type:=original cho driver ouster.")
        return

    t_off, t_dt = off['t']
    if t_dt != 6:      # 6 = UINT32
        print(f"CẢNH BÁO: 't' có datatype={t_dt}, mong đợi 6 (UINT32)")

    buf = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
    t_raw = buf[:, t_off:t_off + 4].copy().view(np.uint32).ravel()
    t_valid = t_raw[t_raw > 0]

    tmin, tmax = int(t_valid.min()), int(t_valid.max())
    span = tmax - tmin
    print(f"trường 't': min={tmin:,}  max={tmax:,}  khoảng={span:,}")

    # 1024x10 -> mỗi scan 100 ms
    scan_ms = 100.0
    for unit, name, div in [(0, 'giây', 1), (1, 'mili giây', 1e3),
                            (2, 'micro giây', 1e6), (3, 'nano giây', 1e9)]:
        ms = span / div * 1000.0
        mark = "  <<< KHỚP" if 0.5 * scan_ms < ms < 1.5 * scan_ms else ""
        print(f"   nếu timestamp_unit={unit} ({name:11s}) -> scan dài {ms:12.3f} ms{mark}")
    print(f"   (một scan @1024x10 phải là ~{scan_ms:.0f} ms)")
    print()

    # --- 2. Khoảng trống giữa các scan --------------------------------------
    stamps = np.array([c.header.stamp.sec + c.header.stamp.nanosec * 1e-9 for c in clouds])
    gaps = np.diff(stamps) * 1000.0
    print(f"khoảng giữa các scan (ms): trung bình {gaps.mean():.2f}, "
          f"lệch chuẩn {gaps.std():.2f}, max {gaps.max():.2f}")
    if gaps.max() > 150:
        print("   CẢNH BÁO: có scan bị mất (khoảng > 150 ms)")
    print()

    # --- 3. Jitter của dấu thời gian IMU ------------------------------------
    if len(imu_stamps) > 20:
        ig = np.diff(np.array(imu_stamps)) * 1000.0
        print(f"khoảng giữa các mẫu IMU (ms): trung bình {ig.mean():.3f} "
              f"(lý tưởng 10.000), lệch chuẩn {ig.std():.3f}, max {ig.max():.3f}")
        if ig.std() > 1.0:
            print("   CẢNH BÁO: jitter IMU cao. TIME_FROM_ROS_TIME dùng thời điểm")
            print("   NHẬN gói làm mốc nên chịu ảnh hưởng của mạng. Thử đổi sang")
            print("   TIME_FROM_INTERNAL_OSC: lidar và IMU dùng chung đồng hồ của")
            print("   sensor, nhất quán tuyệt đối với nhau.")
        else:
            print("   OK, jitter thấp.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
