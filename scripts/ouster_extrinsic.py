#!/usr/bin/env python3
"""
Tính extrinsic LiDAR -> IMU cho FAST-LIO từ metadata của sensor Ouster.

BỐI CẢNH TOÁN HỌC
-----------------
Ouster có 3 hệ toạ độ: os_sensor (vỏ máy), os_lidar (tâm quay tia), os_imu (chip IMU).
Metadata cho 2 ma trận 4x4 (row-major, tịnh tiến theo MILIMÉT):

    lidar_to_sensor_transform = T_S_L   (điểm trong os_lidar -> os_sensor)
    imu_to_sensor_transform   = T_S_I   (điểm trong os_imu   -> os_sensor)

FAST-LIO cần phép biến đổi từ hệ LiDAR sang hệ IMU:

    p_imu = extrinsic_R * p_lidar + extrinsic_T
    =>  T_I_L = inv(T_S_I) @ T_S_L

VÌ SAO KHÔNG CHÉP SỐ TỪ DATASHEET
---------------------------------
Vị trí IMU khác nhau giữa các đời (Rev6/Rev7) và các dòng (OS0/OS1/OS2), và mỗi
máy có sai số hiệu chuẩn riêng đã nạp sẵn trong metadata. Lệch 1 cm hoặc lệch
trục là đủ để FAST-LIO trôi thấy rõ sau vài chục mét.

VÌ SAO DÙNG os_lidar CHỨ KHÔNG PHẢI os_sensor
---------------------------------------------
Driver ouster_ros mặc định point_cloud_frame = os_lidar, và ở chế độ đó nó
KHÔNG áp lidar_to_sensor_transform lên point cloud. Nên điểm nhận được nằm
trong hệ os_lidar. Đổi sang os_sensor thì dùng cờ --frame sensor.

CÁCH DÙNG
---------
    python3 scripts/ouster_extrinsic.py 192.168.123-metadata.json
    python3 scripts/ouster_extrinsic.py --host 192.168.123.40
"""

import argparse
import json
import sys
import urllib.request

import numpy as np


def find_key(obj, key):
    """Tìm `key` ở bất kỳ độ sâu nào.

    Metadata đổi cấu trúc theo firmware: FW 2.x để phẳng ở gốc, FW 3.x gói vào
    'imu_intrinsics'/'lidar_intrinsics'. Máy bạn chạy FW 3.1.0 nên thuộc nhóm sau.
    """
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        for v in obj.values():
            found = find_key(v, key)
            if found is not None:
                return found
    return None


def as_matrix(flat16, name):
    if flat16 is None or len(flat16) != 16:
        sys.exit(f"LỖI: không tìm thấy (hoặc sai kích thước) '{name}' trong metadata.")
    m = np.array(flat16, dtype=float).reshape(4, 4)
    m[:3, 3] /= 1000.0          # metadata dùng mm, ROS dùng m
    return m


def quat_from_matrix(R):
    """Ma trận xoay 3x3 -> quaternion (x, y, z, w), phương pháp Shepperd.

    Chọn nhánh theo phần tử đường chéo lớn nhất để tránh chia cho số gần 0 khi
    góc xoay gần 180 độ — đúng trường hợp Ouster, vì os_lidar xoay đúng 180 độ
    quanh trục Z so với os_imu.
    """
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def fmt_matrix(R):
    rows = []
    for idx, r in enumerate(R):
        pad = "" if idx == 0 else " " * 26
        rows.append(pad + ", ".join(f"{v: .8f}" for v in r))
    return ",\n".join(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("metadata", nargs="?", help="đường dẫn file metadata JSON")
    g.add_argument("--host", help="IP sensor, lấy metadata qua HTTP API")
    ap.add_argument("--frame", choices=["lidar", "sensor"], default="lidar",
                    help="khớp với param point_cloud_frame của driver")
    args = ap.parse_args()

    if args.host:
        url = f"http://{args.host}/api/v1/sensor/metadata"
        print(f"# Tải {url}", file=sys.stderr)
        with urllib.request.urlopen(url, timeout=5) as r:
            meta = json.loads(r.read().decode())
    else:
        with open(args.metadata) as f:
            meta = json.load(f)

    T_S_I = as_matrix(find_key(meta, "imu_to_sensor_transform"), "imu_to_sensor_transform")
    T_S_L = as_matrix(find_key(meta, "lidar_to_sensor_transform"), "lidar_to_sensor_transform")

    T_I_S = np.linalg.inv(T_S_I)
    T_I_X = T_I_S @ T_S_L if args.frame == "lidar" else T_I_S

    R, t = T_I_X[:3, :3], T_I_X[:3, 3]

    prod = find_key(meta, "prod_line") or "?"
    sn = find_key(meta, "prod_sn") or "?"
    fw = find_key(meta, "build_rev") or "?"
    beams = find_key(meta, "beam_altitude_angles")

    print(f"# Sensor        : {prod}   SN {sn}   FW {fw}")
    print(f"# Số tia (ring) : {len(beams) if beams else '?'}  -> preprocess.scan_line")
    print(f"# Point cloud ở : os_{args.frame}")
    print("#")
    print("# ==== Dán vào mục `mapping:` của yaml FAST-LIO ====")
    print()
    print(f"            extrinsic_T: [ {t[0]: .8f}, {t[1]: .8f}, {t[2]: .8f} ]")
    print(f"            extrinsic_R: [ {fmt_matrix(R)} ]")
    print()

    # Static TF nối cây FAST-LIO (camera_init -> body) vào cây driver Ouster.
    # body CHÍNH LÀ os_imu, nhưng os_imu đã có cha là os_sensor và TF2 cấm
    # một frame có 2 cha -> nối body -> os_sensor bằng T_I_S = inv(T_S_I).
    q = quat_from_matrix(T_I_S[:3, :3])
    ts = T_I_S[:3, 3]
    print("# ==== Static TF: body -> os_sensor ====")
    print(f"# {ts[0]:.9f} {ts[1]:.9f} {ts[2]:.9f} "
          f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} body os_sensor")
    print()
    print(f"BODY_TO_OS_SENSOR_XYZ  = ({ts[0]:.9f}, {ts[1]:.9f}, {ts[2]:.9f})")
    print(f"BODY_TO_OS_SENSOR_QUAT = ({q[0]:.9f}, {q[1]:.9f}, {q[2]:.9f}, {q[3]:.9f})")
    print()

    err = np.abs(R @ R.T - np.eye(3)).max()
    det = np.linalg.det(R)
    print(f"# kiểm tra |R·Rᵀ-I|max = {err:.2e}   (phải ~0)", file=sys.stderr)
    print(f"# kiểm tra det(R)      = {det:+.9f}  (phải ~ +1)", file=sys.stderr)
    print(f"# khoảng cách LiDAR-IMU = {np.linalg.norm(t)*1000:.2f} mm", file=sys.stderr)


if __name__ == "__main__":
    main()
