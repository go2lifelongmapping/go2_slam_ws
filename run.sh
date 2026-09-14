#!/usr/bin/env bash
# ============================================================================
#  go2_slam — script khởi động
#
#    ./run.sh              SLAM đầy đủ (driver + FAST-LIO + rviz)
#    ./run.sh view         chỉ xem lidar thô trong rviz, không chạy SLAM
#    ./run.sh rec <tên>    ghi rosbag
#    ./run.sh play <tên>   phát lại bag + chạy SLAM (không cần lidar)
#    ./run.sh shell        mở shell trong container
#    ./run.sh stop         dừng mọi node ROS (container vẫn chạy)
#
#  Tuỳ chọn: đặt biến LIDAR_IP để bỏ qua bước dò mDNS
#    LIDAR_IP=169.254.1.2 ./run.sh
# ============================================================================
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

DDS_URI="file:///ws/docker/cyclonedds.xml"

# --- Chạy một lệnh trong container, đã nạp sẵn môi trường ROS ---------------
# Dùng bash -lc thì .bashrc thoát sớm (shell không tương tác), nên source tay.
in_container() {
    docker compose exec -e CYCLONEDDS_URI="$1" slam \
        bash -c "source /opt/ros/foxy/setup.bash && source /ws/install/setup.bash && $2"
}

ensure_container() {
    if ! docker compose ps --status running --format '{{.Name}}' | grep -q go2_slam; then
        echo ">>> Khởi động container..."
        docker compose up -d
        sleep 2
    fi
}

allow_x11() {
    # Container mở cửa sổ rviz2 qua socket X11 của host. Không có dòng này thì
    # rviz báo: qt.qpa.xcb: could not connect to display :0
    xhost +local: >/dev/null 2>&1 || true
}

# --- Dò IP lidar qua mDNS ---------------------------------------------------
# IP link-local (169.254.x.x) ĐỔI mỗi lần cắm lại cáp, nên không hardcode được.
# Container không phân giải được .local (thiếu libnss-mdns), nên phải dò ở host
# rồi truyền IP vào.
find_lidar() {
    if [ -n "${LIDAR_IP:-}" ]; then echo "$LIDAR_IP"; return; fi
    local ip
    ip=$(timeout 8 avahi-browse -rtp _ouster-lidar._tcp 2>/dev/null \
         | awk -F';' '$1=="=" && $3=="IPv4" {print $8; exit}')
    if [ -z "$ip" ]; then
        echo ">>> KHÔNG dò được lidar qua mDNS." >&2
        echo "    Kiểm tra: cáp đã cắm chưa (cat /sys/class/net/eno1/carrier)," >&2
        echo "    lidar đã cấp nguồn 24V chưa." >&2
        echo "    Hoặc chỉ định tay:  LIDAR_IP=x.x.x.x ./run.sh" >&2
        exit 1
    fi
    echo "$ip"
}

check_eno1() {
    # cyclonedds.xml ghim vào eno1. Card down thì Cyclone không bind được và
    # mọi node chết với: rcl node's rmw handle is invalid
    if [ "$(cat /sys/class/net/eno1/carrier 2>/dev/null || echo 0)" != "1" ]; then
        echo ">>> eno1 không có cáp. Với chế độ offline (play), bỏ qua CycloneDDS." >&2
        return 1
    fi
    return 0
}

CMD="${1:-slam}"

case "$CMD" in
  slam)
    ensure_container; allow_x11
    IP="$(find_lidar)"
    echo ">>> Lidar: $IP"
    echo ">>> Chạy SLAM đầy đủ. Ctrl-C để dừng (bản đồ lưu vào src/FAST_LIO/PCD/)."
    in_container "$DDS_URI" \
      "ros2 launch go2_slam go2_slam.launch.py sensor_hostname:=$IP"
    ;;

  view)
    ensure_container; allow_x11
    IP="$(find_lidar)"
    echo ">>> Lidar: $IP  — chỉ xem dữ liệu thô, KHÔNG chạy SLAM"
    in_container "$DDS_URI" \
      "ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=$IP lidar_mode:=1024x10 timestamp_mode:=TIME_FROM_ROS_TIME viz:=true"
    ;;

  rec)
    ensure_container
    IP="$(find_lidar)"
    NAME="${2:-lab_$(date +%Y%m%d_%H%M%S)}"
    echo ">>> Lidar: $IP"
    echo ">>> Ghi vào bags/$NAME  (~0.94 GB mỗi phút)"
    echo ">>> ĐỨNG YÊN 2-3 giây trước khi di chuyển: FAST-LIO cần dữ liệu tĩnh"
    echo "    lúc đầu để ước lượng trọng lực và bias của IMU."
    # Driver chạy nền, rosbag chạy tiền cảnh để Ctrl-C dừng đúng chỗ.
    docker compose exec -d -e CYCLONEDDS_URI="$DDS_URI" slam bash -c \
      "source /opt/ros/foxy/setup.bash && source /ws/install/setup.bash && \
       ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=$IP \
         lidar_mode:=1024x10 timestamp_mode:=TIME_FROM_ROS_TIME viz:=false > /tmp/drv.log 2>&1"
    echo ">>> Đợi driver sẵn sàng..."
    for _ in $(seq 1 25); do
        sleep 2
        if docker compose exec -T slam bash -c 'grep -q "set_config service created" /tmp/drv.log 2>/dev/null'; then break; fi
    done
    in_container "$DDS_URI" \
      "ros2 bag record --qos-profile-overrides-path /ws/config/rosbag_qos.yaml \
         -o /ws/bags/$NAME /ouster/points /ouster/imu /ouster/metadata /tf_static"
    ;;

  play)
    ensure_container; allow_x11
    NAME="${2:-}"
    if [ -z "$NAME" ]; then
        echo "Dùng: ./run.sh play <tên_bag>"; echo "Có sẵn:"; ls -1 bags/ 2>/dev/null; exit 1
    fi
    echo ">>> Phát lại bags/$NAME + chạy SLAM (không cần lidar)"
    echo ">>> Mở cửa sổ thứ hai và chạy:  ./run.sh shell  rồi:"
    echo "    ros2 bag play /ws/bags/$NAME --qos-profile-overrides-path /ws/config/rosbag_qos.yaml"
    # Không dùng CycloneDDS config: nó ghim eno1, mà chế độ offline có thể rút cáp.
    in_container "" "ros2 launch go2_slam fastlio.launch.py"
    ;;

  shell)
    ensure_container; allow_x11
    # Shell TƯƠNG TÁC: .bashrc chạy đầy đủ nên ROS + CYCLONEDDS_URI + alias cb
    # đã có sẵn, không cần source tay.
    docker compose exec slam bash
    ;;

  stop)
    docker compose exec -T slam bash -c \
      'pkill -f "[o]s_driver"; pkill -f "[f]astlio"; pkill -f "[r]viz2"; pkill -f "[s]tatic_transform"; pkill -f "[r]osbag"' \
      2>/dev/null || true
    echo ">>> Đã dừng các node ROS. Container vẫn chạy."
    ;;

  *)
    sed -n '3,17p' "$0" | sed 's/^# \?//'
    exit 1
    ;;
esac
