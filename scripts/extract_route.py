#!/usr/bin/env python3
"""
Trích tuyến đường từ quỹ đạo FAST-LIO thành danh sách waypoint.

Đọc src/FAST_LIO/Log/mat_pre.txt — FAST-LIO ghi mỗi scan một dòng, 25 cột —
rồi lọc thành chuỗi waypoint thưa hơn, dùng làm tuyến cho robot bám theo.


CÁCH DÙNG
    python3 scripts/extract_route.py
    python3 scripts/extract_route.py --spacing 0.5 --ang-step 10
    python3 scripts/extract_route.py --traj routes/traj_khuD_17-09.txt -o routes/khuD.csv
"""

import argparse
import os

import numpy as np


def load_trajectory(path):
    """Đọc thời gian, yaw và vị trí. usecols là 0-indexed nên lệch 1 so với bảng trên."""
    d = np.loadtxt(path, usecols=(0, 3, 4, 5, 6))
    t = d[:, 0]
    yaw = np.radians(d[:, 1])      # cột 4 trong file, đơn vị ĐỘ -> radian
    xyz = d[:, 2:5]                # cột 5, 6, 7
    return t, xyz, yaw


def angdiff(a, b):
    d = a - b
    return np.arctan2(np.sin(d), np.cos(d))


def resample(xyz, yaw, spacing, ang_step):
    keep = [0]
    for i in range(1, len(xyz)):
        moved = np.linalg.norm(xyz[i] - xyz[keep[-1]])
        turned = abs(angdiff(yaw[i], yaw[keep[-1]]))
        if moved >= spacing or turned >= ang_step:
            keep.append(i)
    if keep[-1] != len(xyz) - 1:        # luôn giữ điểm cuối
        keep.append(len(xyz) - 1)
    return np.array(keep)


def max_unrepresented_turn(yaw, k):
    out = []
    for a, b in zip(k[:-1], k[1:]):
        seg = np.unwrap(yaw[a:b + 1])
        out.append(np.degrees(np.abs(seg - seg[0]).max()))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--traj', default='src/FAST_LIO/Log/mat_pre.txt')
    ap.add_argument('--spacing', type=float, default=1.0,
                    help='khoảng cách tối thiểu giữa hai waypoint, mét')
    ap.add_argument('--ang-step', type=float, default=15.0,
                    help='giữ thêm waypoint khi robot xoay quá ngần này ĐỘ')
    ap.add_argument('-o', '--out', default='routes/route.csv')
    args = ap.parse_args()

    if not os.path.exists(args.traj):
        raise SystemExit(f'không thấy {args.traj}\n'
                         f'mat_pre.txt bị ghi đè mỗi lần FAST-LIO khởi động — '
                         f'chạy lại bag, hoặc trỏ --traj vào bản đã chép ra.')

    t, xyz, yaw = load_trajectory(args.traj)
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    net_turn = np.degrees(abs(np.unwrap(yaw)[-1] - yaw[0]))
    print(f'quỹ đạo gốc : {len(xyz)} điểm, {t[-1]-t[0]:.0f} s, '
          f'{step.sum():.1f} m, xoay ròng {net_turn:.0f}°')

    # ang_step nhập vào là ĐỘ cho dễ đọc, nhưng yaw trong code là RADIAN.
    # Quên np.radians ở đây thì ngưỡng thành 15 rad = 859°, không bao giờ đạt,
    # và hàm chạy y như chưa có vế xoay — mà không báo lỗi gì.
    k = resample(xyz, yaw, args.spacing, np.radians(args.ang_step))
    t2, xyz2, yaw2 = t[k], xyz[k], yaw[k]

    turns = max_unrepresented_turn(yaw, k)
    gaps = np.linalg.norm(np.diff(xyz2, axis=0), axis=1)
    print(f'sau khi lọc  : {len(k)} waypoint '
          f'(cách >= {args.spacing} m hoặc xoay >= {args.ang_step}°)')
    print(f'  khoảng cách giữa 2 waypoint : trung vị {np.median(gaps):.2f} m, '
          f'lớn nhất {gaps.max():.2f} m')
    print(f'  khúc cua không được mô tả   : lớn nhất {turns.max():.0f}°, '
          f'{(turns > 45).sum()} đoạn vượt 45°')

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        f.write('# t,x,y,z,yaw_rad\n')
        for ti, p, y in zip(t2, xyz2, yaw2):
            f.write(f'{ti:.3f},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{y:.4f}\n')
    print(f'>>> đã ghi {args.out}')

    print(f'điểm cuối cách điểm đầu: {np.linalg.norm(xyz2[-1]-xyz2[0]):.2f} m')


if __name__ == '__main__':
    main()
