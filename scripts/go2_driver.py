#!/usr/bin/env python3
"""Cau noi: nhan lenh van toc qua UDP -> SportClient.Move() @ 20 Hz.

Chay tren HOST robot, KHONG source ROS (xung dot libddsc 0.7 vs pip cyclonedds).
    python3 scripts/go2_driver.py --dry-run     # chi in, khong dieu khien
    python3 scripts/go2_driver.py               # dieu khien that
"""
import argparse, socket, struct, time

UDP_HOST, UDP_PORT = "0.0.0.0", 43210
RATE        = 20.0    # Hz gui lai lenh (watchdog robot ~1 s)
CMD_TIMEOUT = 0.3     # s khong co lenh moi -> dung
VX_MAX, VY_MAX, VYAW_MAX = 0.5, 0.3, 0.8   # gioi han an toan

def clamp(v, m):
    return max(-m, min(m, v))

ap = argparse.ArgumentParser()
ap.add_argument("--dry-run", action="store_true", help="chi in, khong goi Move")
ap.add_argument("--iface", default="eth0")
args = ap.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_HOST, UDP_PORT))
sock.setblocking(False)

sport = None
if not args.dry_run:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    ChannelFactoryInitialize(0, args.iface)
    sport = SportClient()
    sport.SetTimeout(10.0)
    sport.Init()

mode = "KHO (dry-run)" if args.dry_run else "THAT"
print("go2_driver | che do %s | nghe UDP %d | Ctrl+C de dung" % (mode, UDP_PORT))

vx = vy = vyaw = 0.0
t_last  = 0.0
stopped = True
period  = 1.0 / RATE
n = 0

try:
    while True:
        # vet het datagram dang cho, chi giu cai moi nhat
        while True:
            try:
                data, _ = sock.recvfrom(64)
            except BlockingIOError:
                break
            if len(data) == 24:
                a, b, c = struct.unpack("<ddd", data)
                vx   = clamp(a, VX_MAX)
                vy   = clamp(b, VY_MAX)
                vyaw = clamp(c, VYAW_MAX)
                t_last = time.time()

        fresh = (time.time() - t_last) < CMD_TIMEOUT

        if fresh:
            if sport:
                if stopped:
                    # StopMove dua robot ve mode 0 -> khong nhan Move nua.
                    # Move dung _CallNoReply nen KHONG bao loi: robot dung im
                    # ma log van in Move(...) nhu dang chay binh thuong.
                    # Chi goi mot lan moi dot, khong lap o 20 Hz.
                    sport.BalanceStand()
                    time.sleep(0.1)
                    print("  BalanceStand -> san sang nhan lenh")
                sport.Move(vx, vy, vyaw)
            stopped = False
            n += 1
            if n % 10 == 0:
                print("  Move(%5.2f, %5.2f, %5.2f)" % (vx, vy, vyaw))
        elif not stopped:
            # het lenh moi -> dung mot lan roi im
            if sport:
                sport.StopMove()
            print("  het lenh -> StopMove")
            vx = vy = vyaw = 0.0
            stopped = True
            n = 0

        time.sleep(period)

except KeyboardInterrupt:
    print("\ndang dung...")
finally:
    if sport:
        sport.StopMove()
    sock.close()
    print("da dung")
