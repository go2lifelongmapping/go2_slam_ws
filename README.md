# go2_slam_ws

LiDAR-Inertial SLAM cho **Unitree Go2** dùng **Ouster OS1-32** và **FAST-LIO2**,
chạy trong Docker với ROS 2 Foxy.

Mục đích của Docker: Go2 EDU chạy Ubuntu 20.04 + ROS 2 Foxy, còn máy phát triển
thường là Ubuntu 22.04 (Humble). Container Foxy giữ cho môi trường khớp với robot.

## Đã kiểm chứng trên

| | |
|---|---|
| LiDAR | Ouster **OS-1-32-U2-SR** (Rev7, firmware **v3.1.0**) |
| Máy chủ | Ubuntu 22.04, Docker 29.5, 16 nhân / 14 GB RAM, GPU AMD |
| Container | Ubuntu 20.04 + ROS 2 Foxy, CycloneDDS 0.7.0 |
| SLAM | FAST-LIO2 (`hku-mars/FAST_LIO`, nhánh `ROS2`) |
| Driver | `ouster-lidar/ouster-ros` v0.12.7, nhánh `ros2-foxy` |

Kết quả đo được: `/ouster/points` **10.0 Hz**, `/ouster/imu` **100 Hz**,
`/cloud_registered` **10.0 Hz**.

## Bắt đầu

```bash
git clone <repo-url> go2_slam_ws && cd go2_slam_ws

./scripts/fetch_src.sh          # kéo ouster-ros + FAST_LIO, áp patch
docker compose build            # dựng image Foxy (~10 phút)
docker compose up -d
docker compose exec slam bash -c 'source /opt/ros/foxy/setup.bash && cd /ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

./run.sh                        # chạy SLAM
```

Một việc phải làm trên host, nếu không point cloud tụt xuống ~4.6 Hz:

```bash
echo 'net.core.rmem_max=26214400
net.core.rmem_default=26214400' | sudo tee /etc/sysctl.d/60-ouster-lidar.conf
sudo sysctl --system
```

## Sử dụng

| Lệnh | Việc |
|---|---|
| `./run.sh` | SLAM đầy đủ — driver + FAST-LIO + rviz |
| `./run.sh view` | chỉ xem lidar thô, không chạy SLAM |
| `./run.sh rec <tên>` | ghi rosbag vào `bags/` |
| `./run.sh play <tên>` | phát lại bag + SLAM, không cần lidar |
| `./run.sh shell` | shell trong container |
| `./run.sh stop` | dừng mọi node ROS |

Script tự dò IP lidar qua mDNS — IP link-local đổi mỗi lần cắm lại cáp.
Chỉ định tay: `LIDAR_IP=169.254.1.2 ./run.sh`

## Cấu trúc

```
docker/          Dockerfile (Foxy), entrypoint.sh, cyclonedds.xml
config/          rosbag_qos.yaml — QoS override cho rosbag2
patches/         3 bản vá bắt buộc cho FAST_LIO + giải thích
scripts/         fetch_src.sh, ouster_extrinsic.py, probe_topics.py
src/go2_slam/    package chính: config FAST-LIO, launch, rviz
src/livox_ros_driver2/   stub message-only (xem bên dưới)
```

`src/FAST_LIO/` và `src/ouster-ros/` nằm trong `.gitignore` — dùng
`scripts/fetch_src.sh` để kéo về.

## Sáu cái bẫy và cách xử lý

Ghi lại để người sau không mất thời gian như lần đầu.

### 1. Nhánh sai của ouster-ros

Nhánh mặc định `ros2` chỉ hỗ trợ Humble trở lên. Phải dùng **`ros2-foxy`**.

### 2. FAST_LIO bắt buộc `livox_ros_driver2` kể cả khi dùng Ouster

`CMakeLists.txt:62` có `find_package(livox_ros_driver2 REQUIRED)`. Giải pháp:
package **stub chỉ có định nghĩa message** (`src/livox_ros_driver2/`, ~40 dòng).
Không cần Livox-SDK2. Code Livox trong FAST_LIO không bao giờ chạy vì
`laserMapping.cpp:921` rẽ nhánh theo `lidar_type`, và ta đặt `lidar_type: 3`.

### 3. QoS IMU không tương thích — lỗi im lặng, nghiêm trọng nhất

`laserMapping.cpp:929` subscribe IMU với QoS mặc định (**RELIABLE**), trong khi
driver publish **BEST_EFFORT**. FAST-LIO không nhận được mẫu IMU nào →
`/cloud_registered` im lặng ở 0 Hz. Build sạch, node chạy, không lỗi rõ ràng.

Đã vá trong `patches/`. Xem `patches/README.md` cho cả 3 sửa đổi.

### 4. Bộ đệm nhận UDP mặc định quá nhỏ

Mỗi scan là ~410 KB nhưng `net.core.rmem_max` mặc định chỉ 208 KB → mất hơn
một nửa số scan. Xem phần "Bắt đầu".

### 5. `launch_arguments` rò rỉ ra phạm vi cha

`IncludeLaunchDescription` đặt launch configuration ở phạm vi cha. Truyền
`rviz:=false` cho một include sẽ tắt luôn node rviz khai báo phía sau trong
cùng `LaunchDescription`. Sửa bằng `GroupAction([...], scoped=True)`.

### 6. `ros2 topic hz` của Foxy không đọc được BEST_EFFORT

Foxy chưa có cờ `--qos-reliability` (chỉ từ Galactic). Lệnh này treo im lặng
trên mọi topic sensor. Dùng `scripts/probe_topics.py` để đo.

Tương tự với `ros2 bag record` — cần `config/rosbag_qos.yaml`, nếu không bag rỗng.

## Extrinsic IMU ↔ LiDAR

**Không chép số từ datasheet.** Vị trí IMU khác nhau giữa các đời và các dòng,
và mỗi máy có sai số hiệu chuẩn riêng nạp sẵn trong metadata.

```bash
python3 scripts/ouster_extrinsic.py --host <IP_lidar>
```

Script tính `T_I_L = inv(imu_to_sensor) @ lidar_to_sensor` rồi in ra khối
`extrinsic_T` / `extrinsic_R` để dán vào `src/go2_slam/config/ouster_os1_32.yaml`,
kèm static TF `body -> os_sensor` cho launch file.

## Giới hạn

FAST-LIO2 là **odometry**, không có loop closure. Đi vòng lớn quay về chỗ cũ,
hai đầu bản đồ sẽ không khớp. Cần loop closure thì xem `FAST_LIO_SLAM` hoặc LIO-SAM.

Đừng đặt `publish.map_en: true`: `publish_map()` cộng dồn vào `pcl_wait_pub`
mà không bao giờ xoá, rồi publish toàn bộ mỗi giây — sau 5 phút là ~230 MB/giây.
Muốn xem bản đồ tích luỹ, dùng **Decay Time** của rviz trên `/cloud_registered`
(đã đặt 300 giây trong `go2_slam.rviz`); muốn lưu, dùng `pcd_save_en`.

## Giấy phép

Cấu hình và script trong repo này: MIT.
`FAST_LIO` (GPL-2.0) và `ouster-ros` (BSD-3-Clause) giữ giấy phép gốc của chúng.
