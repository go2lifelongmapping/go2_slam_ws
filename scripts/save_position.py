#!/usr/bin/env python3
"""Ghi quỹ đạo FAST-LIO ra CSV t,x,y. Chạy TRONG container, khi SLAM đang chạy.

Thay cho src/FAST_LIO/Log/mat_pre.txt vì:
  - /Odometry là tư thế SAU hiệu chỉnh Kalman, mat_pre.txt là TRƯỚC
  - mat_pre.txt bị GHI ĐÈ mỗi lần FAST-LIO khởi động; file này do bạn đặt tên

    python3 /ws/scripts/save_position.py /ws/routes/traj_khuC.csv
    python3 /ws/scripts/save_position.py /ws/routes/traj_khuC.csv --route /ws/routes/route_khuC.csv
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class SavePosition(Node):

    def __init__(self, out, spacing):
        super().__init__("save_position")
        self.f = open(out, "w")
        self.f.write("# t,x,y\n")
        self.spacing = spacing
        self.t0 = None
        self.n = 0
        self.dist = 0.0
        self.last = None          # vị trí lần trước, để cộng dồn quãng đường
        self.poses = []           # giữ lại để lọc waypoint lúc thoát
        self.create_subscription(Odometry, "/Odometry", self.on_odom, 50)
        self.get_logger().info("ghi vào %s — Ctrl+C để dừng" % out)

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        p = msg.pose.pose.position

        if self.last is not None:
            self.dist += math.hypot(p.x - self.last[0], p.y - self.last[1])
        self.last = (p.x, p.y)

        row = (t - self.t0, p.x, p.y)
        self.f.write("%.3f,%.4f,%.4f\n" % row)
        self.f.flush()            # flush từng dòng: Ctrl+C giữa chừng vẫn còn dữ liệu
        self.poses.append(row)

        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info("%d tư thế | %.0f s | %.1f m | (%+.1f, %+.1f)"
                                   % (self.n, row[0], self.dist, p.x, p.y))

    def xuat_waypoint(self, path):
        """Lọc thưa theo khoảng cách, cùng quy tắc với extract_route.py."""
        keep = [0]
        for i in range(1, len(self.poses)):
            a, b = self.poses[keep[-1]], self.poses[i]
            if math.hypot(b[1] - a[1], b[2] - a[2]) >= self.spacing:
                keep.append(i)
        if keep[-1] != len(self.poses) - 1:
            keep.append(len(self.poses) - 1)
        with open(path, "w") as g:
            g.write("# t,x,y\n")
            for i in keep:
                g.write("%.3f,%.4f,%.4f\n" % self.poses[i])
        return len(keep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="CSV quỹ đạo đầy đủ 10 Hz")
    ap.add_argument("--route", help="xuất thêm file waypoint đã lọc thưa")
    ap.add_argument("--spacing", type=float, default=1.0,
                    help="khoảng cách tối thiểu giữa hai waypoint, mét")
    args = ap.parse_args()

    rclpy.init()
    node = SavePosition(args.out, args.spacing)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        print("\n>>> %s — %d tư thế, %.1f m" % (args.out, node.n, node.dist))
        if args.route and node.poses:
            k = node.xuat_waypoint(args.route)
            print(">>> %s — %d waypoint (cách >= %.1f m)" % (args.route, k, args.spacing))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

