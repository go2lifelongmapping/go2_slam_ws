#!/usr/bin/env python3
"""
Trích tuyến đường từ quỹ đạo FAST-LIO thành danh sách waypoint.

CÁCH DÙNG
    python3 scripts/extract_route.py
    python3 scripts/extract_route.py --spacing 0.5
    python3 scripts/extract_route.py --traj routes/traj_khuD_17-09.txt -o routes/khuD.csv
"""

import argparse
import os

import numpy as np


def load_trajectory(path):
    """Đọc thời gian và vị trí x, y. usecols là 0-indexed nên lệch 1 so với file."""
    d = np.loadtxt(path, usecols=(0, 4, 5))
    t = d[:, 0]
    xy = d[:, 1:3]                 # cột 5, 6 trong file
    return t, xy


def resample(xy, spacing):
    keep = [0]
    for i in range(1, len(xy)):
        if np.linalg.norm(xy[i] - xy[keep[-1]]) >= spacing:
            keep.append(i)
    if keep[-1] != len(xy) - 1:        # luôn giữ điểm cuối
        keep.append(len(xy) - 1)
    return np.array(keep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--traj', default='src/FAST_LIO/Log/mat_pre.txt')
    ap.add_argument('--spacing', type=float, default=1.0,
                    help='khoảng cách tối thiểu giữa hai waypoint, mét')
    ap.add_argument('-o', '--out', default='routes/route.csv')
    args = ap.parse_args()

    if not os.path.exists(args.traj):
        raise SystemExit(f'không thấy {args.traj}\n'
                         f'mat_pre.txt bị ghi đè mỗi lần FAST-LIO khởi động — '
                         f'chạy lại bag, hoặc trỏ --traj vào bản đã chép ra.')

    t, xy = load_trajectory(args.traj)
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    print(f'quỹ đạo gốc : {len(xy)} điểm, {t[-1]-t[0]:.0f} s, {step.sum():.1f} m')

    k = resample(xy, args.spacing)
    t2, xy2 = t[k], xy[k]

    gaps = np.linalg.norm(np.diff(xy2, axis=0), axis=1)
    print(f'sau khi lọc  : {len(k)} waypoint (cách >= {args.spacing} m)')
    print(f'  khoảng cách giữa 2 waypoint : trung vị {np.median(gaps):.2f} m, '
          f'lớn nhất {gaps.max():.2f} m')

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as f:
        f.write('# t,x,y\n')
        for ti, p in zip(t2, xy2):
            f.write(f'{ti:.3f},{p[0]:.4f},{p[1]:.4f}\n')
    print(f'>>> đã ghi {args.out}')

    print(f'điểm cuối cách điểm đầu: {np.linalg.norm(xy2[-1]-xy2[0]):.2f} m')


if __name__ == '__main__':
    main()
