#!/usr/bin/env bash
# ============================================================================
#  Cầu nối rt/lowstate (Unitree, domain 0) -> /joint_states (ROS, domain 42).
#  CHẠY TRÊN ROBOT.
#
#  Hai tiến trình nối bằng pipe, mỗi bên một môi trường — bắt buộc phải thế:
#  gói `cyclonedds` pip của SDK Unitree build cho CycloneDDS đời mới, còn ROS
#  Foxy mang CycloneDDS 0.7. Source ROS xong thì SDK gãy ngay lúc import với
#  `undefined symbol: ddsi_sertype_v0`.
#
#  Vế trái chạy TRƯỚC khi source ROS nên giữ môi trường sạch; vế phải source
#  ROS trong subshell riêng.
#
#    ./scripts/lowstate_bridge.sh [iface] [rate_hz] [domain]
# ============================================================================
# KHÔNG dùng `set -u`: các script setup của ROS tham chiếu biến chưa đặt
# (AMENT_TRACE_SETUP_FILES, COLCON_TRACE...) nên source chúng dưới `set -u` là
# subshell chết ngay, và vế trái của pipe gãy với BrokenPipeError.
set -o pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IFACE="${1:-eth0}"
RATE="${2:-50}"
DOMAIN="${3:-42}"

echo ">>> rt/lowstate ($IFACE, domain 0)  ->  /joint_states (domain $DOMAIN, ${RATE} Hz)"
python3 "$WS/scripts/lowstate_reader.py" "$IFACE" "$RATE" \
  | ( set +u
      source /opt/ros/foxy/setup.bash
      [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
      export ROS_DOMAIN_ID="$DOMAIN"
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      unset CYCLONEDDS_URI
      exec python3 "$WS/scripts/joint_state_pub.py" )
