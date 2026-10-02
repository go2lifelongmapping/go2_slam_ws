#!/usr/bin/env python3
"""Gui lenh van toc toi go2_driver qua UDP.

    python3 scripts/send_vel.py <vx> <vy> <vyaw> [dur]
    python3 scripts/send_vel.py 0.15 0 0 2      # di thang 0.15 m/s trong 2 s
    python3 scripts/send_vel.py 0 0 0.3 2       # xoay 0.3 rad/s trong 2 s
"""
import socket, struct, sys, time

if len(sys.argv) < 4:
    print(__doc__)
    sys.exit(1)

vx, vy, vyaw = (float(a) for a in sys.argv[1:4])
dur = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0

if dur > 5.0:
    print("dur > 5 s -> chan lai de an toan")
    sys.exit(1)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pkt = struct.pack("<ddd", vx, vy, vyaw)

print("gui vx=%.2f vy=%.2f vyaw=%.2f trong %.1f s | Ctrl+C de dung" %
      (vx, vy, vyaw, dur))
t0 = time.time()
try:
    while time.time() - t0 < dur:
        s.sendto(pkt, ("127.0.0.1", 43210))
        time.sleep(0.05)
finally:
    s.close()
    print("ngung gui -> driver se StopMove sau 0.3 s")
