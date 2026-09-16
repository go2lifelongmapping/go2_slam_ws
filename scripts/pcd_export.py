#!/usr/bin/env python3
"""
Xuất bản đồ scans.pcd sang định dạng mang đi được.

VÌ SAO CẦN
----------
FAST-LIO ghi ra PCD nhị phân với 8 trường (x y z intensity normal_x normal_y
normal_z curvature), 32 byte mỗi điểm. Bản đồ khu D là 49.857.942 điểm = 1,6 GB.
Ba vấn đề: quá nặng để mở, 4 trường normal/curvature là rác nội bộ của FAST-LIO
(luôn bằng 0 hoặc vô nghĩa với người xem), và PCD thì ít phần mềm đọc được.

Script này gộp voxel, bỏ trường thừa, rồi ghi ra PLY hoặc PCD.
PLY mở được bằng CloudCompare, MeshLab, Blender, Open3D — PCD thì hầu như chỉ PCL.

CÁCH DÙNG
    python3 scripts/pcd_export.py                          # -> maps/map.ply, voxel 5 cm
    python3 scripts/pcd_export.py --voxel 0.02 -o map_min.ply
    python3 scripts/pcd_export.py --format pcd -o map.pcd
    python3 scripts/pcd_export.py --zmin -0.5 --zmax 2.5   # cắt trần và sàn
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcd_view import read_pcd, voxel_downsample          # noqa: E402


def write_ply(path, xyz, inten):
    """PLY nhị phân little-endian. Intensity thành màu xám để xem được ngay."""
    n = len(xyz)
    if inten is not None:
        lo, hi = np.percentile(inten, [2, 98])
        g = np.clip((inten - lo) / max(hi - lo, 1e-9), 0, 1)
        rgb = (g * 255).astype(np.uint8)
        dt = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                       ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
        arr = np.empty(n, dtype=dt)
        arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        arr['red'] = arr['green'] = arr['blue'] = rgb
        props = ('property float x\nproperty float y\nproperty float z\n'
                 'property uchar red\nproperty uchar green\nproperty uchar blue\n')
    else:
        dt = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        arr = np.empty(n, dtype=dt)
        arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        props = 'property float x\nproperty float y\nproperty float z\n'

    with open(path, 'wb') as f:
        f.write(f'ply\nformat binary_little_endian 1.0\n'
                f'element vertex {n}\n{props}end_header\n'.encode('ascii'))
        arr.tofile(f)


def write_pcd(path, xyz, inten):
    """PCD nhị phân, chỉ x/y/z/intensity — bỏ 4 trường nội bộ của FAST-LIO."""
    n = len(xyz)
    if inten is not None:
        dt = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
        fields, sizes, types, counts = 'x y z intensity', '4 4 4 4', 'F F F F', '1 1 1 1'
    else:
        dt = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        fields, sizes, types, counts = 'x y z', '4 4 4', 'F F F', '1 1 1'
    arr = np.empty(n, dtype=dt)
    arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if inten is not None:
        arr['intensity'] = inten
    with open(path, 'wb') as f:
        f.write(f'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
                f'FIELDS {fields}\nSIZE {sizes}\nTYPE {types}\nCOUNT {counts}\n'
                f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {n}\nDATA binary\n'.encode('ascii'))
        arr.tofile(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', default='src/FAST_LIO/PCD/scans.pcd')
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--format', choices=['ply', 'pcd'], default=None)
    ap.add_argument('--voxel', type=float, default=0.05, help='cạnh ô voxel, mét; 0 = không gộp')
    ap.add_argument('--zmin', type=float, default=None, help='bỏ điểm thấp hơn (cắt sàn)')
    ap.add_argument('--zmax', type=float, default=None, help='bỏ điểm cao hơn (cắt trần)')
    args = ap.parse_args()

    out = args.out or ('maps/map.' + (args.format or 'ply'))
    fmt = args.format or (out.rsplit('.', 1)[-1].lower())
    if fmt not in ('ply', 'pcd'):
        sys.exit(f'định dạng không hỗ trợ: {fmt}')

    print(f'đọc {args.pcd} ...')
    arr, n = read_pcd(args.pcd)
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)
    inten = arr['intensity'].astype(np.float32) if 'intensity' in arr.dtype.names else None
    print(f'  {n:,} điểm')

    keep = np.isfinite(xyz).all(axis=1)
    if args.zmin is not None:
        keep &= xyz[:, 2] >= args.zmin
    if args.zmax is not None:
        keep &= xyz[:, 2] <= args.zmax
    if not keep.all():
        xyz, inten = xyz[keep], (inten[keep] if inten is not None else None)
        print(f'  sau khi lọc: {len(xyz):,} điểm')

    if args.voxel > 0:
        xyz, inten = voxel_downsample(xyz, inten, args.voxel)
        print(f'  sau voxel {args.voxel} m: {len(xyz):,} điểm')

    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    (write_ply if fmt == 'ply' else write_pcd)(out, xyz, inten)
    mb = os.path.getsize(out) / 1e6
    print(f'>>> đã ghi {out}  ({len(xyz):,} điểm, {mb:.0f} MB)')
    b = xyz.max(axis=0) - xyz.min(axis=0)
    print(f'    kích thước bản đồ: {b[0]:.1f} x {b[1]:.1f} x {b[2]:.1f} m')


if __name__ == '__main__':
    main()
