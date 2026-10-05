#!/usr/bin/env python3
"""Node ROS: /Odometry (FAST-LIO) -> pure pursuit -> UDP -> go2_driver.

Chạy TRONG container (ROS Foxy, domain 42), go2_driver chạy trên host.

    python3 /ws/scripts/pp_node.py /ws/routes/route_short.csv --dry-run
    python3 /ws/scripts/pp_node.py /ws/routes/route_short.csv
"""
import argparse
import math
import os
import socket
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pure_pursuit import PurePursuit, load_route        # noqa: E402

UDP_ADDR = ("127.0.0.1", 43210)
JUMP_MAX = 1.0      # m giữa hai bản tin 10 Hz = 10 m/s -> chắc chắn FAST-LIO hỏng


def quat_to_yaw(q):
    """Quaternion -> góc yaw. Rút gọn cho trường hợp phẳng."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PPNode(Node):

    def __init__(self, route, dry_run=False, log_path=None, **kw):
        super().__init__("pure_pursuit_node")
        self.pp = PurePursuit(route, **kw)
        self.dry_run = dry_run
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stopped = False
        self.last_xy = None
        self.n = 0
        # mat_pre.txt cua FAST-LIO bi GHI DE moi lan no khoi dong lai, nen
        # ghi quy dao o day de khong mat du lieu sau moi lan thu.
        self.log = open(log_path, "w") if log_path else None
        if self.log:
            self.log.write("t,x,y,yaw,alpha,vx,vyaw,i\n")
        self.t0 = time.time()
        self.create_subscription(Odometry, "/Odometry", self.on_odom, 10)
        self.get_logger().info(
            "bám %d waypoint | chế độ %s | gửi UDP %d"
            % (len(route), "KHÔ (dry-run)" if dry_run else "THẬT", UDP_ADDR[1]))

    def halt(self, why):
        """Ngừng gửi. go2_driver sẽ StopMove sau 0.3 s im lặng."""
        if not self.stopped:
            self.stopped = True
            self.get_logger().warn("NGỪNG: %s" % why)

    def on_odom(self, msg):
        if self.stopped:
            return

        p = msg.pose.pose.position
        x, y = p.x, p.y

        # FAST-LIO từng phân kỳ tới 8e8 m -> chặn trước khi lệnh ra chân robot
        if self.last_xy is not None:
            jump = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if jump > JUMP_MAX:
                self.halt("vị trí nhảy %.1f m trong một bản tin — FAST-LIO hỏng" % jump)
                return
        self.last_xy = (x, y)

        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx, vyaw, done, info = self.pp.step(x, y, yaw)

        if done:
            self.halt("ĐẾN ĐÍCH")
            return

        if not self.dry_run:
            self.sock.sendto(struct.pack("<ddd", vx, 0.0, vyaw), UDP_ADDR)

        if self.log:
            self.log.write("%.3f,%.4f,%.4f,%.5f,%.5f,%.3f,%.3f,%d\n"
                           % (time.time() - self.t0, x, y, yaw,
                              info["alpha"], vx, vyaw, info["i"]))
            self.log.flush()

        self.n += 1
        if self.n % 10 == 0:
            self.get_logger().info(
                "wp %3d/%d | còn %6.2f m | alpha %+6.1f° | vx %.2f | vyaw %+.2f"
                % (info["i"], len(self.pp.pts), info["d_goal"],
                   math.degrees(info["alpha"]), vx, vyaw))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route")
    ap.add_argument("--dry-run", action="store_true",
                    help="chỉ in, không gửi UDP — robot không động")
    ap.add_argument("--lookahead", type=float, default=0.8)
    ap.add_argument("--v-nom", type=float, default=0.20)
    ap.add_argument("--k-ang", type=float, default=1.5)
    ap.add_argument("--turn-in-place", type=float, default=0.7)
    ap.add_argument("--log", help="ghi quy dao ra CSV de phan tich sau")
    args = ap.parse_args()

    route = load_route(args.route)

    rclpy.init()
    node = PPNode(route, dry_run=args.dry_run, log_path=args.log,
                  lookahead=args.lookahead, v_nom=args.v_nom,
                  k_ang=args.k_ang, turn_in_place=args.turn_in_place)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C — ngừng gửi")
    finally:
        if node.log:
            node.log.close()
            print("da ghi %s" % args.log)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
