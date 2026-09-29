#!/usr/bin/env python3
"""
Mở file bản đồ .pcd của FAST-LIO bằng Open3D.

CHẠY BẰNG MÔI TRƯỜNG RIÊNG, không phải python3 của hệ thống:
    ~/.venv_pcl/bin/python scripts/view_map.py

VÌ SAO CẦN VENV RIÊNG: hệ thống có numpy 2.x, còn scipy mà open3d kéo về lại
build cho numpy 1.x -> `numpy.core.multiarray failed to import`. Hạ numpy hệ
thống sẽ làm hỏng thứ khác, nên để open3d sống trong ~/.venv_pcl.

VÌ SAO PHẢI GỘP VOXEL: scans.pcd của FAST-LIO chứa MỌI điểm của MỌI scan, không
khử trùng lặp. Đo trên bản đồ lab: 27.570.733 điểm, nhưng gộp voxel 5 cm chỉ còn
405.740 — tức 98,5% là điểm trùng chỗ, vì robot quét đi quét lại cùng bề mặt.
Vẽ thẳng 27 triệu điểm sẽ giật hoặc treo.

CÁCH DÙNG
    ~/.venv_pcl/bin/python scripts/view_map.py
    ~/.venv_pcl/bin/python scripts/view_map.py --voxel 0.02        # net hon
    ~/.venv_pcl/bin/python scripts/view_map.py --voxel 0           # khong gop
    ~/.venv_pcl/bin/python scripts/view_map.py --zmin -1 --zmax 2  # cat tran/san
    ~/.venv_pcl/bin/python scripts/view_map.py --save map.ply      # xuat, khong mo cua so

⚠️ MÀU KHÔNG HIỆN TRÊN MÁY NÀY
Điểm sẽ vẽ ra ĐEN dù script có gán màu. Đã kiểm chứng: render offscreen cho ra
đúng 1 màu duy nhất [0,0,0]; ép point_color_option sang Color hoặc ZCoordinate
thì KHÔNG vẽ được gì (0 pixel). Đó là lỗi biên dịch shader của trình vẽ cũ
Open3D trên nền Mesa/amdgpu, không phải lỗi script.

File xuất bằng --save VẪN CÓ màu, nên mở bằng CloudCompare hay MeshLab sẽ thấy.
Muốn xem có màu ngay: sudo apt install cloudcompare

ĐIỀU KHIỂN TRONG CỬA SỔ
    chuột trái kéo : xoay        chuột phải kéo : tịnh tiến
    lăn chuột      : phóng to    R : đặt lại góc nhìn
    +/-            : đổi cỡ điểm  Q hoặc Esc : thoát
"""

import argparse
import os
import sys

import numpy as np
import open3d as o3d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', default='src/FAST_LIO/PCD/scans.pcd')
    ap.add_argument('--voxel', type=float, default=0.05, help='cạnh ô voxel, mét; 0 = không gộp')
    ap.add_argument('--zmin', type=float, default=None, help='bỏ điểm thấp hơn (cắt sàn)')
    ap.add_argument('--zmax', type=float, default=None, help='bỏ điểm cao hơn (cắt trần)')
    ap.add_argument('--trim', type=float, default=99.5,
                    help='giữ lại phân vị này của khoảng cách ngang, cắt điểm lạc; 100 = không cắt')
    ap.add_argument('--save', default=None, help='ghi ra .ply/.pcd rồi thoát, không mở cửa sổ')
    args = ap.parse_args()

    if not os.path.exists(args.pcd):
        sys.exit(f'không thấy {args.pcd}\n'
                 f'Bản đồ chỉ được ghi khi Ctrl-C node FAST-LIO (nhờ patches/0002).')

    print(f'đọc {args.pcd} ...')
    pc = o3d.io.read_point_cloud(args.pcd)
    n0 = len(pc.points)
    print(f'  {n0:,} điểm')

    if args.zmin is not None or args.zmax is not None:
        p = np.asarray(pc.points)
        keep = np.ones(len(p), bool)
        if args.zmin is not None:
            keep &= p[:, 2] >= args.zmin
        if args.zmax is not None:
            keep &= p[:, 2] <= args.zmax
        pc = pc.select_by_index(np.flatnonzero(keep))
        print(f'  sau khi cắt z: {len(pc.points):,} điểm')

    # Cắt điểm lạc TRƯỚC khi vẽ. Không có bước này, vài chục nghìn điểm ở rìa
    # (phản xạ sai, nhiễu tầm xa) kéo hộp bao ra gấp nhiều lần kích thước thật,
    # Open3D căn khung nhìn theo hộp bao đó, và toàn bộ bản đồ co lại thành một
    # cục bé xíu giữa màn hình. Đo trên bản đồ lab: 99% điểm trong bán kính
    # 8,0 m nhưng p99.9 đã là 30,4 m và p100 là 36,2 m.
    if args.trim < 100:
        q = np.asarray(pc.points)
        r = np.linalg.norm(q[:, :2], axis=1)
        lim = np.percentile(r, args.trim)
        keep = r <= lim
        if not keep.all():
            pc = pc.select_by_index(np.flatnonzero(keep))
            print(f'  cắt điểm lạc ngoài bán kính {lim:.1f} m (phân vị {args.trim}): '
                  f'bỏ {int((~keep).sum()):,} điểm, còn {len(pc.points):,}')

    if args.voxel > 0:
        pc = pc.voxel_down_sample(args.voxel)
        print(f'  sau voxel {args.voxel} m: {len(pc.points):,} điểm '
              f'(bỏ {100*(1-len(pc.points)/n0):.1f}% điểm trùng chỗ)')

    b = np.asarray(pc.points)
    ext = b.max(axis=0) - b.min(axis=0)
    print(f'  kích thước bản đồ: {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} m')

    # Tô màu theo độ cao — bản đồ FAST-LIO không có màu sẵn, và không tô thì
    # Open3D vẽ toàn điểm xám/đen, rất khó đọc hình khối.
    # Làm TRƯỚC khi lưu để file xuất ra cũng có màu.
    z = b[:, 2]
    span = float(z.max() - z.min())
    t = (z - z.min()) / max(span, 1e-9)
    pc.colors = o3d.utility.Vector3dVector(
        np.stack([t, 0.4 + 0.4 * (1 - t), 1 - t], axis=1))
    print(f'  tô màu theo độ cao: z từ {z.min():.2f} đến {z.max():.2f} m')

    if args.save:
        o3d.io.write_point_cloud(args.save, pc)
        print(f'>>> đã ghi {args.save} ({os.path.getsize(args.save)/1e6:.0f} MB)')
        return

    print('>>> mở cửa sổ. Q hoặc Esc để thoát.')
    o3d.visualization.draw_geometries([pc], window_name=os.path.basename(args.pcd),
                                      width=1400, height=900)


if __name__ == '__main__':
    main()
