#!/usr/bin/env python3
"""Pure pursuit: bám theo route.csv. Lõi thuần — không ROS, không DDS.

    python3 scripts/pure_pursuit.py routes/route.csv --test
    python3 scripts/pure_pursuit.py routes/route.csv --sim
    python3 scripts/pure_pursuit.py routes/route.csv --sim --deficit 0.58
"""
import argparse
import math

import numpy as np


def angdiff(a, b):
    """Hiệu góc a - b, luôn nằm trong (-pi, pi]."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def load_route(path):
    """route.csv (t,x,y) -> mảng Nx2 (x, y).

    Bỏ t: robot đi nhanh chậm tùy nó, không bám theo thời gian cũ.
    Không cần yaw: luật lái suy ra góc quay từ VỊ TRÍ các waypoint.
    """
    d = np.loadtxt(path, delimiter=",", comments="#")
    return d[:, 1:3]


class PurePursuit:
    """Sinh lệnh (vx, vyaw) để bám theo chuỗi waypoint.

    lookahead     m    điểm ngắm cách robot bao xa; lớn = mượt nhưng cắt góc
    v_nom         m/s  tốc độ khi đường thẳng
    k_ang         -    hệ số bù góc; >1 vì robot chỉ quay ~58% so với lệnh
    turn_in_place rad  lệch hơn mức này thì dừng lại quay tại chỗ
    goal_tol      m    coi như đã tới đích
    v_max, w_max       trần cuối; go2_driver còn chặn lần nữa ở 0.5 / 0.8
    window        -    số waypoint tìm về phía trước, chống nhảy cóc
    """

    def __init__(self, route, lookahead=0.8, v_nom=0.25, k_ang=1.5,
                 turn_in_place=0.7, goal_tol=0.3,
                 v_max=0.4, w_max=0.8, window=40):
        self.pts = np.asarray(route, dtype=float)
        self.lookahead = lookahead
        self.v_nom = v_nom
        self.k_ang = k_ang
        self.turn_in_place = turn_in_place
        self.goal_tol = goal_tol
        self.v_max = v_max
        self.w_max = w_max
        self.window = window
        self.i = 0          # waypoint đang bám — CHỈ TIẾN, không lùi

    def _advance(self, x, y):
        """Dời self.i tới waypoint gần (x, y) nhất, chỉ tìm về phía trước."""
        hi = min(len(self.pts), self.i + self.window)
        seg = self.pts[self.i:hi]
        j = int(np.argmin(np.hypot(seg[:, 0] - x, seg[:, 1] - y)))
        self.i += j
        return self.i

    def _lookahead_point(self, i, x, y):
        """Waypoint đầu tiên, tính từ i, cách (x, y) ít nhất lookahead mét."""
        for k in range(i, len(self.pts)):
            if math.hypot(self.pts[k, 0] - x, self.pts[k, 1] - y) >= self.lookahead:
                return k
        return len(self.pts) - 1        # gần cuối tuyến thì ngắm luôn điểm cuối

    def step(self, x, y, yaw):
        """Một chu kỳ điều khiển. Trả về (vx, vyaw, done, info)."""
        i = self._advance(x, y)
        d_goal = math.hypot(self.pts[-1, 0] - x, self.pts[-1, 1] - y)

        if i >= len(self.pts) - 1 and d_goal < self.goal_tol:
            return 0.0, 0.0, True, {"i": i, "k": i, "d_goal": d_goal, "alpha": 0.0}

        k = self._lookahead_point(i, x, y)
        lx, ly = self.pts[k]
        alpha = angdiff(math.atan2(ly - y, lx - x), yaw)

        if abs(alpha) > self.turn_in_place:
            # lệch quá nhiều: đứng yên quay cho thẳng hướng đã
            vx = 0.0
            vyaw = self.k_ang * alpha
        else:
            vx = self.v_nom * math.cos(alpha)
            L = max(1e-3, math.hypot(lx - x, ly - y))
            # luật pure pursuit: độ cong cung tròn = 2 sin(alpha) / L
            vyaw = self.k_ang * 2.0 * vx * math.sin(alpha) / L

        vx = max(-self.v_max, min(self.v_max, vx))
        vyaw = max(-self.w_max, min(self.w_max, vyaw))
        return vx, vyaw, False, {"i": i, "k": k, "d_goal": d_goal, "alpha": alpha}


def cross_track(pts, x, y):
    """Khoảng cách tới ĐƯỜNG GẤP KHÚC, không phải tới waypoint gần nhất.

    Đo tới waypoint sẽ bị thổi phồng bởi bước rời rạc 1 m giữa các điểm.
    """
    a, b = pts[:-1], pts[1:]
    ab = b - a
    ap = np.array([x, y]) - a
    denom = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12)
    t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return float(np.min(np.hypot(proj[:, 0] - x, proj[:, 1] - y)))


def simulate(route, deficit=1.0, dt=0.1, t_max=1800.0, **kw):
    """Mô phỏng xe đơn bánh ở 10 Hz. deficit = góc quay THẬT / góc quay LỆNH."""
    pp = PurePursuit(route, **kw)
    x, y = route[0]
    yaw = math.atan2(route[1, 1] - route[0, 1], route[1, 0] - route[0, 0])
    traj, errs, t = [], [], 0.0
    while t < t_max:
        vx, vyaw, done, _ = pp.step(x, y, yaw)
        if done:
            break
        yaw += vyaw * deficit * dt
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        t += dt
        traj.append((t, x, y, yaw, vx, vyaw))
        errs.append(cross_track(route, x, y))
    return done, t, np.array(traj), np.array(errs), pp


def run_tests(route):
    pp = PurePursuit(route)
    bad = 0
    for n, (x, y) in enumerate(route):
        if pp._advance(x, y) != n:
            bad += 1
    print("bám đúng chỉ số : %d/%d" % (len(route) - bad, len(route)))

    pp2 = PurePursuit(route)
    print("ở toạ độ wp 143, chỉ số đang 0 -> bám wp %d (phải < 40)"
          % pp2._advance(route[143, 0], route[143, 1]))

    pp3 = PurePursuit(route)
    pp3.i = 70
    print("lệch 2 m tại wp 70 -> bám wp %d" % pp3._advance(route[70, 0] + 2.0, route[70, 1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route")
    ap.add_argument("--test", action="store_true", help="kiểm tra _advance")
    ap.add_argument("--sim", action="store_true", help="mô phỏng chạy hết tuyến")
    ap.add_argument("--deficit", type=float, default=1.0,
                    help="tỉ lệ góc quay thật/lệnh (đo được trên Go2: 0.58)")
    ap.add_argument("--lookahead", type=float, default=0.8)
    ap.add_argument("--v-nom", type=float, default=0.25)
    ap.add_argument("--k-ang", type=float, default=1.5)
    ap.add_argument("--turn-in-place", type=float, default=0.7)
    ap.add_argument("-o", "--out", help="ghi quỹ đạo mô phỏng ra CSV")
    args = ap.parse_args()

    route = load_route(args.route)
    seglen = np.hypot(np.diff(route[:, 0]), np.diff(route[:, 1]))
    print("route : %d waypoint, dài %.1f m" % (len(route), seglen.sum()))
    print("bước  : min %.2f m, p50 %.2f m, max %.2f m"
          % (seglen.min(), np.median(seglen), seglen.max()))

    if args.test:
        run_tests(route)
    if not args.sim:
        return

    done, t, traj, errs, pp = simulate(
        route, deficit=args.deficit, lookahead=args.lookahead,
        v_nom=args.v_nom, k_ang=args.k_ang, turn_in_place=args.turn_in_place)

    print("\ndeficit        : %.2f" % args.deficit)
    print("tới đích       : %s  (waypoint %d/%d)"
          % ("CÓ" if done else "KHÔNG", pp.i + 1, len(route)))
    print("thời gian      : %.0f s  (%.1f phút)" % (t, t / 60))
    if len(errs):
        print("sai lệch ngang : tb %.3f m | p95 %.3f m | max %.3f m"
              % (errs.mean(), np.percentile(errs, 95), errs.max()))
        pivot = np.mean(traj[:, 4] == 0.0)
        print("quay tại chỗ   : %.1f%% số bước" % (100 * pivot))
        print("vyaw chạm trần : %.1f%% số bước"
              % (100 * np.mean(np.abs(traj[:, 5]) >= pp.w_max - 1e-9)))
    if args.out:
        np.savetxt(args.out, traj, delimiter=",",
                   header="t,x,y,yaw,vx,vyaw", comments="# ")
        print("đã ghi %s" % args.out)


if __name__ == "__main__":
    main()
