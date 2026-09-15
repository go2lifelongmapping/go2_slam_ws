#!/usr/bin/env python3
"""
Đọc IMU trong rosbag2 và cho biết LIDAR có di chuyển hay không.

Vì sao cần: tên bag nói về vật thể trong cảnh, không nói lidar đứng hay đi.
Muốn chỉnh tham số cho hiện tượng "tường nhân đôi khi di chuyển" thì bag phải
chứa đúng chuyển động đó.

Đọc trực tiếp file .db3 (SQLite) rồi giải tuần tự bằng rclpy — không cần phát lại.

Hai đại lượng:
  |gyro|      tốc độ góc, rad/s. Đứng yên ~0.01; xoay khi đi bộ 0.3-1.0.
  |acc| - g   độ lệch gia tốc so với trọng lực, m/s^2. Đứng yên ~0.1;
              bước chân 1-3.
"""

import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

G = 9.80665


def read_imu(db_path, topic='/ouster/imu'):
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    row = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        names = [r[0] for r in cur.execute("SELECT name FROM topics")]
        sys.exit(f"Không có topic {topic}. Có: {names}")
    tid = row[0]
    out = []
    for ts, data in cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        m = deserialize_message(bytes(data), Imu)
        out.append((ts * 1e-9,
                    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    con.close()
    return np.array(out)


def main():
    for path in sys.argv[1:]:
        name = path.rstrip('/').split('/')[-1]
        d = read_imu(path)
        t = d[:, 0] - d[0, 0]
        gyro = np.linalg.norm(d[:, 1:4], axis=1)
        accn = np.linalg.norm(d[:, 4:7], axis=1)
        dev = np.abs(accn - G)

        print(f"=== {name} ===")
        print(f"  {len(d)} mẫu IMU, {t[-1]:.1f} s")
        print(f"  |gyro|     trung bình {gyro.mean():6.3f}  p95 {np.percentile(gyro,95):6.3f}  max {gyro.max():6.3f} rad/s")
        print(f"  |acc|-g    trung bình {dev.mean():6.3f}  p95 {np.percentile(dev,95):6.3f}  max {dev.max():6.3f} m/s²")

        # Chia thành các đoạn 5 giây, đánh dấu đoạn nào có chuyển động
        print("  Dòng thời gian (mỗi ô 5 giây):")
        bins = np.arange(0, t[-1] + 5, 5)
        idx = np.digitize(t, bins) - 1
        bar = ""
        for b in range(len(bins) - 1):
            sel = idx == b
            if not sel.any():
                bar += " "
                continue
            g = gyro[sel].mean()
            bar += "." if g < 0.05 else ("-" if g < 0.20 else ("+" if g < 0.50 else "#"))
        print(f"    [{bar}]")
        print("     . đứng yên   - động nhẹ   + đi bộ   # xoay mạnh")

        moving = (gyro > 0.20).mean() * 100
        print(f"  => {moving:.0f}% thời lượng có chuyển động rõ rệt")
        if moving < 10:
            print("     LIDAR GẦN NHƯ ĐỨNG YÊN -> bag này KHÔNG dùng để chỉnh")
            print("     hiện tượng nhân đôi khi di chuyển được.")
        else:
            print("     Bag có chuyển động -> DÙNG ĐƯỢC để chỉnh tham số.")
        print()


if __name__ == '__main__':
    main()
