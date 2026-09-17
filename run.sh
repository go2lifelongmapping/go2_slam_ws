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

# --- Chọn file compose ------------------------------------------------------
# Jetson của Go2 (aarch64) phải dùng docker-compose.go2.yml: không /dev/dri,
# không X11. `docker compose` tự đọc biến COMPOSE_FILE nên chỉ cần export.
if [ -z "${COMPOSE_FILE:-}" ] && [ "$(uname -m)" = "aarch64" ] \
   && [ -f "$WS/docker-compose.go2.yml" ]; then
    export COMPOSE_FILE="docker-compose.go2.yml"
fi
# User trên Go2 chưa chắc là uid 1000; build image cho khớp để khỏi lỗi quyền
# ghi trên volume /ws.
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

# --- Có màn hình không? -----------------------------------------------------
# Go2 không có X server. Mở rviz2 ở đó vừa đầy log lỗi vừa ngốn CPU của Jetson
# — đúng cái nút thắt đã đo: rviz làm FAST-LIO tụt từ 9.1 xuống 3.1 scan/s.
# Xem bản đồ thì chạy rviz2 trên laptop và để nó subscribe qua mạng.
if [ -z "${DISPLAY:-}" ]; then RVIZ=false; else RVIZ=true; fi

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

# --- Driver Ouster đã chạy sẵn chưa? ----------------------------------------
# Hai node os_driver cùng lúc sẽ tranh nhau cấu hình sensor qua HTTP API và
# giành port UDP 7502/7503 -> cả hai đều hỏng. Nên chế độ 'rec' phải dùng lại
# driver đang chạy thay vì khởi động thêm.
driver_running() {
    # pgrep -x khớp ĐÚNG TÊN tiến trình, không phải dòng lệnh.
    # Dùng -f "/os_driver" là sai: nó khớp cả dòng lệnh của chính shell đang
    # chạy lệnh đó (vì chuỗi "/os_driver" nằm trong đấy) -> luôn trả về TRUE.
    # Cùng cái bẫy khiến `pkill -f os_driver; pkill -f fastlio` tự giết mình.
    docker compose exec -T slam bash -c 'pgrep -x os_driver >/dev/null' 2>/dev/null
}

# --- Liệt kê bag có thật ----------------------------------------------------
# `ls bags/` là SAI: bag hay nằm sâu trong thư mục con (vd 14/09/26_vattinh),
# nên nó chỉ in ra "14" — một cái tên mà `./run.sh play` không dùng được.
# Thư mục bag = thư mục chứa metadata.yaml.
list_bags() {
    find bags -name metadata.yaml -printf '%h\n' 2>/dev/null \
        | sed 's|^bags/||' | sort
}

# Thời lượng bag, đọc từ metadata.yaml (nanogiây) để in cho người dùng biết
# phải đợi bao lâu.
bag_duration() {
    awk '/^  duration:/{f=1;next} f&&/nanoseconds:/{printf "%.0f giây\n", $2/1e9; exit}' \
        "bags/$1/metadata.yaml" 2>/dev/null || echo "?"
}

# --- Sinh cyclonedds config với ĐÚNG card mạng ------------------------------
# docker/cyclonedds.xml ghim cứng 'eno1' — đúng cho laptop, SAI cho Go2 (Jetson
# thường là eth0). Cyclone không bind được card không tồn tại và mọi node chết
# với "rcl node's rmw handle is invalid".
# Chọn card theo thứ tự: $DDS_IFACE  ->  card có route tới lidar  ->  card đầu
# tiên đang có cáp/sóng. Kết quả ghi ra docker/cyclonedds.gen.xml (gitignore).
pick_iface() {
    local ip="${1:-}" n
    if [ -n "${DDS_IFACE:-}" ]; then echo "$DDS_IFACE"; return; fi
    if [ -n "$ip" ]; then
        ip route get "$ip" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -1
        return
    fi
    for d in /sys/class/net/*; do
        n=$(basename "$d")
        case "$n" in lo|docker*|veth*|br-*) continue;; esac
        [ "$(cat "$d/carrier" 2>/dev/null)" = "1" ] && { echo "$n"; return; }
    done
}

# In ra CYCLONEDDS_URI dùng được, hoặc chuỗi rỗng nếu không có card nào.
# Chuỗi rỗng = để Cyclone tự chọn; chấp nhận được MIỄN LÀ mọi node cùng phiên
# đều nhận cùng giá trị này.
dds_uri() {
    local iface; iface="$(pick_iface "${1:-}")"
    if [ -z "$iface" ]; then echo ""; return; fi
    # Neo vào ĐẦU DÒNG: dòng 7 của cyclonedds.xml cũng chứa thẻ này nhưng nằm
    # trong khối comment (sau <General>), thay nhầm sẽ làm hỏng tài liệu.
    sed "s|^\([[:space:]]*\)<NetworkInterfaceAddress>[^<]*</NetworkInterfaceAddress>|\1<NetworkInterfaceAddress>$iface</NetworkInterfaceAddress>|" \
        "$WS/docker/cyclonedds.xml" > "$WS/docker/cyclonedds.gen.xml"

    # DDS_PEERS: danh sách IP cách nhau bởi dấu phẩy, dùng khi mạng CHẶN
    # MULTICAST — hotspot điện thoại là ví dụ điển hình (đã đo: 0/20 gói
    # multicast tới nơi). Không có nó thì discovery chết im lặng: ros2 topic
    # list trống trơn, không một dòng lỗi.
    #   DDS_PEERS=172.20.10.5,172.20.10.13 ./run.sh ...
    if [ -n "${DDS_PEERS:-}" ]; then
        local plist="" a
        local IFS=,
        for a in $DDS_PEERS; do
            plist="$plist        <Peer address=\"$a\"/>\n"
        done
        unset IFS
        sed -i -e "s|<AllowMulticast>[^<]*</AllowMulticast>|<AllowMulticast>false</AllowMulticast>|" \
               -e "s|</Discovery>|      <Peers>\n$plist      </Peers>\n    </Discovery>|" \
               "$WS/docker/cyclonedds.gen.xml"
    fi
    echo "file:///ws/docker/cyclonedds.gen.xml"
}

# --- Mô hình 3D của Go2 trong rviz ------------------------------------------
# robot_state_publisher + joint_zeros.py, phát TF cho 29 link của con chó từ
# base_link trở xuống. Chỉ có ý nghĩa khi có màn hình.
#   $1 = true  -> tự phát cả static TF body->base_link (dùng khi phát lại bag,
#                 vì fastlio.launch.py không phát nó; go2_slam.launch.py thì có)
# Tắt hẳn bằng: ROBOT_MODEL=0 ./run.sh ...
start_robot_model() {
    [ "$RVIZ" = true ] || return 0
    [ "${ROBOT_MODEL:-1}" = "0" ] && return 0
    docker compose exec -d -e CYCLONEDDS_URI="${1:-}" slam bash -c \
      "source /opt/ros/foxy/setup.bash && source /ws/install/setup.bash && \
       ros2 launch go2_slam robot_model.launch.py publish_base_tf:=$2 \
         > /tmp/robot_model.log 2>&1"
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
    start_robot_model "$(dds_uri "$IP")" false
    in_container "$(dds_uri "$IP")" \
      "ros2 launch go2_slam go2_slam.launch.py sensor_hostname:=$IP rviz:=$RVIZ"
    ;;

  view)
    ensure_container; allow_x11
    IP="$(find_lidar)"
    echo ">>> Lidar: $IP  — chỉ xem dữ liệu thô, KHÔNG chạy SLAM"
    in_container "$(dds_uri "$IP")" \
      "ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=$IP lidar_mode:=1024x10 timestamp_mode:=TIME_FROM_ROS_TIME viz:=$RVIZ"
    ;;

  rec)
    ensure_container
    NAME="${2:-lab_$(date +%Y%m%d_%H%M%S)}"

    if driver_running; then
        # Chạy song song với ./run.sh đang mở ở cửa sổ khác.
        # rosbag2 chỉ subscribe, không đụng gì tới sensor -> an toàn.
        echo ">>> Driver đã chạy sẵn, dùng chung (không khởi động thêm)."
    else
        IP="$(find_lidar)"
        echo ">>> Lidar: $IP  — khởi động driver"
        # Driver chạy nền, rosbag chạy tiền cảnh để Ctrl-C dừng đúng chỗ.
        docker compose exec -d -e CYCLONEDDS_URI="$(dds_uri "$IP")" slam bash -c \
          "source /opt/ros/foxy/setup.bash && source /ws/install/setup.bash && \
           ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=$IP \
             lidar_mode:=1024x10 timestamp_mode:=TIME_FROM_ROS_TIME viz:=false > /tmp/drv.log 2>&1"
        echo ">>> Đợi driver sẵn sàng..."
        for _ in $(seq 1 25); do
            sleep 2
            if docker compose exec -T slam bash -c 'grep -q "set_config service created" /tmp/drv.log 2>/dev/null'; then break; fi
        done
    fi

    echo ">>> Ghi vào bags/$NAME  (~0.94 GB mỗi phút)"
    echo ">>> ĐỨNG YÊN 2-3 giây trước khi di chuyển: FAST-LIO cần dữ liệu tĩnh"
    echo "    lúc đầu để ước lượng trọng lực và bias của IMU."
    in_container "$(dds_uri "")" \
      "ros2 bag record --qos-profile-overrides-path /ws/config/rosbag_qos.yaml \
         -o /ws/bags/$NAME /ouster/points /ouster/imu /ouster/metadata /tf_static"
    ;;

  play)
    ensure_container; allow_x11
    NAME="${2:-}"

    if [ -z "$NAME" ]; then
        echo "Dùng: ./run.sh play <tên_bag>"; echo "Có sẵn:"; list_bags; exit 1
    fi
    if [ ! -f "bags/$NAME/metadata.yaml" ]; then
        # Bag nằm sâu trong thư mục con nên tên đầy đủ dài và dễ gõ thiếu.
        # Nếu phần đuôi khớp DUY NHẤT một bag thì dùng luôn, khỏi bắt gõ lại.
        MATCH="$(list_bags | grep -E "(^|/)$(printf '%s' "$NAME" | sed 's/[][\.*^$/]/\\&/g')$" || true)"
        if [ "$(printf '%s\n' "$MATCH" | grep -c .)" = "1" ] && [ -n "$MATCH" ]; then
            echo ">>> '$NAME' -> '$MATCH'"
            NAME="$MATCH"
        else
            echo ">>> Không có bag 'bags/$NAME'." >&2
            [ -n "$MATCH" ] && echo "    (khớp nhiều bag, phải ghi rõ hơn)" >&2
            echo "    Có sẵn:" >&2; list_bags >&2
            exit 1
        fi
    fi

    # FAST-LIO và bag play PHẢI dùng chung một lựa chọn DDS. Lệch nhau thì mỗi
    # bên bám một card mạng khác -> không discovery được -> rviz trống, không
    # một dòng lỗi nào.
    DDS="$(dds_uri "")"

    # Bản đồ của FAST-LIO nằm trong RAM (ikd-tree), không lưu ra đâu cả. Chạy
    # lại lệnh này = bản đồ cũ mất sạch, nên dọn tiến trình cũ cho gọn thay vì
    # để hai node cùng tên tranh topic.
    docker compose exec -T slam bash -c \
      'pkill -f "[f]astlio"; pkill -f "[r]viz2"' 2>/dev/null || true
    sleep 1

    echo ">>> Phát lại bags/$NAME + chạy SLAM (không cần lidar)"
    # FAST-LIO phải sẵn sàng TRƯỚC khi bag chạy: nó không giữ lại scan đến
    # trước lúc subscribe xong, phát sớm là mất mấy giây đầu bag — đúng mấy
    # giây mà nó cần dữ liệu tĩnh để ước lượng trọng lực và bias IMU.
    # FASTLIO_CONFIG: chọn config khác khi bag dùng tên topic khác. Bag ghi qua
    # bridge go2_perception publish /go2/ouster/* chứ không phải /ouster/*, nên
    # config mặc định sẽ không nhận được gì — và FAST-LIO im lặng, không báo lỗi.
    #   FASTLIO_CONFIG=/ws/src/go2_slam/config/go2_bridge_ouster.yaml \
    #     ./run.sh play GO2_KHUD_16-09/khuD_16-09_mau
    CFG_ARG=""
    [ -n "${FASTLIO_CONFIG:-}" ] && CFG_ARG="config_file:=$FASTLIO_CONFIG"
    docker compose exec -d -e CYCLONEDDS_URI="$DDS" slam bash -c \
      "source /opt/ros/foxy/setup.bash && source /ws/install/setup.bash && \
       ros2 launch go2_slam fastlio.launch.py rviz:=$RVIZ $CFG_ARG > /tmp/fastlio.log 2>&1"

    start_robot_model "$DDS" true

    echo ">>> Đợi FAST-LIO sẵn sàng..."
    for _ in $(seq 1 30); do
        sleep 1
        if docker compose exec -T slam bash -c \
             'grep -q "Node init finished" /tmp/fastlio.log 2>/dev/null'; then break; fi
    done
    sleep 2   # rviz2 lên chậm hơn, cho nó kịp subscribe

    echo ">>> Phát bag ($(bag_duration "$NAME")). Ctrl-C để dừng sớm."
    in_container "$DDS" \
      "ros2 bag play /ws/bags/$NAME --qos-profile-overrides-path /ws/config/rosbag_qos.yaml"

    echo ""
    if [ "$RVIZ" = true ]; then
        echo ">>> Bag hết. FAST-LIO và rviz vẫn chạy, bản đồ còn trên màn hình."
    else
        echo ">>> Bag hết. FAST-LIO vẫn chạy (headless, không rviz)."
        echo "    Bản đồ nằm trong RAM; Ctrl-C node để nó ghi ra src/FAST_LIO/PCD/."
    fi
    echo "    Xem log SLAM:  docker compose exec slam tail -f /tmp/fastlio.log"
    echo "    Chạy lại:      ./run.sh play $NAME"
    echo "    Dừng hẳn:      ./run.sh stop"
    ;;

  shell)
    ensure_container; allow_x11
    # Shell TƯƠNG TÁC: .bashrc chạy đầy đủ nên ROS + CYCLONEDDS_URI + alias cb
    # đã có sẵn, không cần source tay.
    docker compose exec slam bash
    ;;

  stop)
    # Dừng cả rosbag: Ctrl-C ở cửa sổ ghi là cách đúng, nhưng lệnh này là
    # phương án dọn dẹp khi có tiến trình treo.
    #
    # PHẢI có "[b]ag play": dòng lệnh thật là `ros2 bag play ...`, KHÔNG chứa
    # chuỗi "rosbag". Trước đây chỉ pkill "[r]osbag" nên bag play không bao giờ
    # bị giết và tích lại sau mỗi lần chạy. Bảy bản sao cùng phát một bag vào
    # cùng topic làm FAST-LIO nhận dữ liệu mâu thuẫn và PHÂN KỲ — vị trí nhảy
    # lên hàng trăm triệu mét. Triệu chứng rất dễ đổ nhầm cho thuật toán.
    docker compose exec -T slam bash -c \
      'pkill -f "[o]s_driver"; pkill -f "[f]astlio"; pkill -f "[r]viz2"; pkill -f "[s]tatic_transform"; pkill -f "[r]obot_state_publisher"; pkill -f "[b]ag play"; pkill -f "[r]osbag"' \
      2>/dev/null || true
    echo ">>> Đã dừng các node ROS. Container vẫn chạy."
    ;;

  *)
    sed -n '3,17p' "$0" | sed 's/^# \?//'
    exit 1
    ;;
esac
