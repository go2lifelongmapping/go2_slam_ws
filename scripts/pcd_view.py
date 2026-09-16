#!/usr/bin/env python3
"""
Phát file .pcd của FAST-LIO ra rviz dưới dạng MỘT message latched.

VÌ SAO CẦN
----------
Trong rviz, `Decay Time` quyết định điểm cũ sống bao lâu; hết hạn là biến mất.
Đặt số thật lớn thì điểm không mất, nhưng rviz phải giữ toàn bộ trong RAM —
bag khu D cho 3.695 scan x ~15.000 điểm = ~55 triệu điểm, quá sức iGPU.

Cách này khác hẳn: bản đồ đầy đủ nằm sẵn trong scans.pcd (49.857.942 điểm cho
bag khu D). Ta gộp voxel xuống mức vẽ được rồi phát MỘT message duy nhất với
durability TRANSIENT_LOCAL. rviz nhận một lần và giữ nguyên — không có gì để
hết hạn, kể cả khi đặt Decay Time = 0.

CÁCH DÙNG
    python3 scripts/pcd_view.py                          # voxel 10 cm
    python3 scripts/pcd_view.py --voxel 0.05             # nét hơn, nặng hơn
    python3 scripts/pcd_view.py --voxel 0.2 --max-points 2000000
"""

import argparse
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

# PCD dùng tên kiểu riêng; ánh xạ sang dtype của numpy.
_T = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2',
      ('U', 4): 'u4', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}


def read_pcd(path):
    """Đọc PCD nhị phân. Chỉ cần x/y/z/intensity nên bỏ qua phần còn lại."""
    with open(path, 'rb') as f:
        hdr, line = {}, b''
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError('PCD hỏng: không thấy DATA')
            s = line.decode('ascii', 'replace').strip()
            k = s.split()[0].upper() if s else ''
            hdr[k] = s.split()[1:]
            if k == 'DATA':
                break
        if hdr['DATA'][0] != 'binary':
            raise RuntimeError(f"chỉ đọc được DATA binary, file này là {hdr['DATA'][0]}")

        names = hdr['FIELDS']
        sizes = [int(x) for x in hdr['SIZE']]
        types = hdr['TYPE']
        counts = [int(x) for x in hdr.get('COUNT', ['1'] * len(names))]
        n = int(hdr['POINTS'][0])

        dt = []
        for nm, sz, ty, ct in zip(names, sizes, types, counts):
            base = _T.get((ty, sz))
            if base is None:
                raise RuntimeError(f'kiểu trường lạ: {ty}{sz}')
            dt.append((nm, base, ct) if ct > 1 else (nm, base))
        arr = np.fromfile(f, dtype=np.dtype(dt), count=n)
    return arr, n


def voxel_downsample(xyz, extra, voxel):
    """Mỗi ô voxel giữ đúng một điểm. Rẻ hơn nhiều so với tính trọng tâm."""
    key = np.floor(xyz / voxel).astype(np.int64)
    # Gộp 3 chỉ số ô thành một khoá duy nhất để dùng np.unique một lần.
    key -= key.min(axis=0)
    mul = np.array([1, key[:, 0].max() + 1,
                    (key[:, 0].max() + 1) * (key[:, 1].max() + 1)], dtype=np.int64)
    flat = key @ mul
    _, idx = np.unique(flat, return_index=True)
    return xyz[idx], (extra[idx] if extra is not None else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', default='/ws/src/FAST_LIO/PCD/scans.pcd')
    ap.add_argument('--topic', default='/map_pcd')
    ap.add_argument('--frame', default='camera_init')
    ap.add_argument('--voxel', type=float, default=0.10, help='cạnh ô voxel, mét')
    ap.add_argument('--max-points', type=int, default=4_000_000,
                    help='nếu sau khi gộp vẫn nhiều hơn, tỉa thưa đều')
    args = ap.parse_args()

    rclpy.init()
    node = Node('pcd_view')
    log = node.get_logger()

    log.info(f'đọc {args.pcd} ...')
    arr, n = read_pcd(args.pcd)
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)
    inten = arr['intensity'].astype(np.float32) if 'intensity' in arr.dtype.names else None

    ok = np.isfinite(xyz).all(axis=1)
    xyz, inten = xyz[ok], (inten[ok] if inten is not None else None)
    log.info(f'{n:,} điểm, bỏ {int((~ok).sum()):,} điểm không hợp lệ')

    xyz, inten = voxel_downsample(xyz, inten, args.voxel)
    log.info(f'sau voxel {args.voxel} m: {len(xyz):,} điểm')

    if len(xyz) > args.max_points:
        step = int(np.ceil(len(xyz) / args.max_points))
        xyz, inten = xyz[::step], (inten[::step] if inten is not None else None)
        log.info(f'tỉa thêm 1/{step}: còn {len(xyz):,} điểm')

    if inten is not None:
        data = np.empty((len(xyz), 4), dtype=np.float32)
        data[:, :3], data[:, 3] = xyz, inten
        fields = ['x', 'y', 'z', 'intensity']
    else:
        data, fields = xyz, ['x', 'y', 'z']

    msg = PointCloud2()
    msg.header.frame_id = args.frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height, msg.width = 1, len(xyz)
    msg.fields = [PointField(name=f, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, f in enumerate(fields)]
    msg.is_bigendian = False
    msg.point_step = 4 * len(fields)
    msg.row_step = msg.point_step * msg.width
    msg.data = data.tobytes()
    msg.is_dense = True

    # TRANSIENT_LOCAL: rviz mở sau vẫn nhận được message đã phát.
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PointCloud2, args.topic, qos)
    pub.publish(msg)
    log.info(f'đã phát {len(xyz):,} điểm lên {args.topic} '
             f'({len(msg.data)/1e6:.0f} MB). Ctrl-C để dừng.')
    log.info('Trong rviz: PointCloud2, Durability Policy = Transient Local, Decay Time = 0')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
