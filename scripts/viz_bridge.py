#!/usr/bin/env python3
"""
Trung chuyển point cloud cho rviz qua đường truyền hẹp (WiFi, hotspot).

VÌ SAO CẦN
----------
/cloud_registered của FAST-LIO là ~15 500 điểm x 48 byte = 725 KB mỗi message,
10 Hz -> 58 Mbit/s. Hotspot điện thoại đo được 3 Mbit/s: không có cửa.

Tệ hơn chuyện băng thông là chuyện PHÂN MẢNH. CycloneDDS cắt message theo
FragmentSize (4000 B), nên 725 KB thành ~186 mảnh. Mất bất kỳ mảnh nào là hỏng
cả message, mà /cloud_registered chạy best_effort nên không gửi lại. Với 4% mất
gói đo được: 0.96^186 ~ 0.05% message tới nguyên vẹn. Gần như không có gì.

CÁCH LÀM
--------
Lấy 1 điểm mỗi `stride`, chỉ giữ x/y/z (12 byte thay vì 48), và hạ xuống `rate`
Hz. Mặc định stride=20, rate=2 -> ~775 điểm, ~9 KB/message, ~150 kbit/s.
9 KB chỉ thành 3 mảnh, nên tỉ lệ tới nguyên vẹn là 0.96^3 ~ 88%.

Đủ để nhìn hình dạng bản đồ và biết robot đang ở đâu. KHÔNG dùng để đánh giá
chất lượng bản đồ — việc đó làm trên file PCD sau khi thu xong.

CÁCH DÙNG (chạy trên ROBOT, cạnh FAST-LIO)
    python3 scripts/viz_bridge.py
    python3 scripts/viz_bridge.py --stride 10 --rate 3     # nét hơn, tốn hơn
"""

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class VizBridge(Node):
    def __init__(self, args):
        super().__init__('viz_bridge')
        self.stride = max(1, args.stride)
        self.period = 1.0 / args.rate if args.rate > 0 else 0.0
        self.last = 0.0
        self.n_in = self.n_out = 0

        self.pub = self.create_publisher(PointCloud2, args.out, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, args.inp, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'{args.inp} -> {args.out}  (1 điểm mỗi {self.stride}, tối đa {args.rate} Hz)')

    def on_cloud(self, msg):
        self.n_in += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.period and (now - self.last) < self.period:
            return
        self.last = now

        n = msg.width * msg.height
        if n == 0:
            return

        # Lấy offset THẬT của x/y/z thay vì giả định 0/4/8 — point_step khác nhau
        # giữa các nguồn (FAST-LIO dùng PointXYZINormal, 48 byte).
        off = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= off.keys():
            self.get_logger().warn('cloud thiếu x/y/z, bỏ qua')
            return

        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        sel = buf[::self.stride]
        if off['y'] == off['x'] + 4 and off['z'] == off['x'] + 8:
            xyz = sel[:, off['x']:off['x'] + 12]          # liền nhau: cắt một nhát
        else:
            xyz = np.concatenate(
                [sel[:, off[c]:off[c] + 4] for c in ('x', 'y', 'z')], axis=1)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(xyz.shape[0])
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = msg.is_bigendian
        out.point_step = 12
        out.row_step = 12 * out.width
        out.data = xyz.tobytes()
        out.is_dense = msg.is_dense
        self.pub.publish(out)
        self.n_out += 1

    def report(self):
        if self.n_in:
            self.get_logger().info(f'nhận {self.n_in}, gửi {self.n_out}')
        self.n_in = self.n_out = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--inp', default='/cloud_registered')
    ap.add_argument('--out', default='/cloud_viz')
    ap.add_argument('--stride', type=int, default=20, help='lấy 1 điểm mỗi N')
    ap.add_argument('--rate', type=float, default=2.0, help='Hz tối đa, 0 = không hạn chế')
    args = ap.parse_args()

    rclpy.init()
    node = VizBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
