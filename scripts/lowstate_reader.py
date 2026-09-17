#!/usr/bin/env python3
"""
Đọc rt/lowstate của Unitree, in ra JSON mỗi dòng. CHẠY TRÊN ROBOT, KHÔNG source ROS.

VÌ SAO TÁCH RIÊNG
-----------------
Gói `cyclonedds` pip mà SDK Unitree dùng được build cho CycloneDDS đời mới. Khi
source ROS Foxy, LD_LIBRARY_PATH đưa CycloneDDS 0.7 của Foxy lên trước và SDK
gãy ngay lúc import:

    ImportError: .../cyclonedds/_clayer...so: undefined symbol: ddsi_sertype_v0

Nên một tiến trình KHÔNG thể vừa dùng SDK vừa dùng rclpy. Script này chỉ đọc và
in ra stdout; scripts/joint_state_pub.py ở đầu kia của pipe lo phần ROS.

Cũng cần biết: `ros2 topic list` trên domain 0 của Go2 sẽ chết
(`bad_alloc caught` liên tục) vì Cyclone 0.7 phân tích sai gói discovery của
Unitree. SDK dùng binding riêng nên không dính.

CÁCH DÙNG
    python3 lowstate_reader.py [eth0] [rate_hz]
"""

import json
import sys
import time

sys.path.insert(0, '/home/unitree/unitree_sdk2_python')
from unitree_sdk2py.core.channel import (ChannelSubscriber,               # noqa: E402
                                         ChannelFactoryInitialize)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_              # noqa: E402

# 12 động cơ đầu của motor_state, theo đúng thứ tự Unitree dùng:
# FR -> FL -> RR -> RL, mỗi chân hip -> thigh -> calf.
# Tên khớp khớp với URDF go2_description.
JOINTS = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
          'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
          'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
          'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else 'eth0'
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    period = 1.0 / rate
    last = [0.0]

    def cb(msg: LowState_):
        # lowstate chạy 500 Hz — quá nhanh cho rviz, hạ xuống `rate`.
        now = time.time()
        if now - last[0] < period:
            return
        last[0] = now
        q = [float(msg.motor_state[i].q) for i in range(12)]
        dq = [float(msg.motor_state[i].dq) for i in range(12)]
        sys.stdout.write(json.dumps({'q': q, 'dq': dq}) + '\n')
        sys.stdout.flush()

    ChannelFactoryInitialize(0, iface)
    sub = ChannelSubscriber('rt/lowstate', LowState_)
    sub.Init(cb, 10)
    # Dòng đầu là tên khớp, để phía kia khỏi phải chép cứng danh sách.
    sys.stdout.write(json.dumps({'names': JOINTS}) + '\n')
    sys.stdout.flush()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
