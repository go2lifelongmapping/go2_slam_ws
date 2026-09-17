# go2_slam — việc cần làm

Cập nhật 17/09/2026. Mọi con số dưới đây đều **đo được**, không phải ước lượng;
chỗ nào là phỏng đoán đều ghi rõ.

Mục tiêu dài hạn: **lifelong SLAM trên Unitree Go2**. Xem phần "Lộ trình" ở cuối.

---

## Hiện trạng

| | |
|---|---|
| SLAM chạy live trên robot | ✔ 10,1 scan/giây trên Jetson, load 0,72 |
| SLAM chạy phát lại bag trên laptop | ✔ |
| Lưu bản đồ ra `scans.pcd` | ✔ (cần patch 0002) |
| Xem từ xa qua WiFi hẹp | ✔ `viz_bridge.py`, 0,10 Mbit/s |
| **Drift trên vòng khép kín** | **✘ 12,33 m / 181 m = 6,81%** |
| Loop closure | ✘ chưa có |
| Bản đồ bền vững, định vị lại | ✘ chưa có |

---

## 1. Việc cần làm ngay

### 1.1 Tách bạch nguyên nhân drift — chạy FAST-LIO trên L1

**Vì sao:** drift 12,33 m trên vòng 181 m là vấn đề lớn nhất hiện nay. Nghi phạm
chính là cloud Ouster trong bag **thiếu trường `time` per-point**, nên FAST-LIO
không khử méo chuyển động được. PCL báo thẳng khi chạy:

```
Failed to find match for field 't'.
```

Bag khu D có sẵn **cả hai lidar, chung một đồng hồ**, nên so sánh được trực tiếp:

```
/go2/ouster/points : 29 405 điểm/khung, 20 byte, KHÔNG có time
/utlidar/cloud     :  3 939 điểm/khung, 32 byte, CÓ time (đơn vị GIÂY)
```

**Làm gì:** tạo config mới trỏ vào `/utlidar/cloud` + `/utlidar/imu`, đặt
`timestamp_unit: 0` (giây), rồi chạy và đo lại drift bằng cách so vị trí đầu–cuối.

**Kết quả quyết định điều gì:**
- Drift tụt xuống dưới 1–2% → nguyên nhân là thiếu `time`. Cách sửa: ghi bag
  thẳng từ driver Ouster, đừng qua bridge (xem 2.1).
- Drift vẫn ~6% → nguyên nhân nằm chỗ khác, và ta đã loại trừ được một khả năng lớn.

**Không tốn phần cứng, không tốn mạng. Dữ liệu đã nằm sẵn trong bag.**

### 1.2 Sửa vị trí lidar theo hiệu chuẩn thật

`go2_slam.launch.py` đang dùng số **ước lượng bằng mắt**:

```python
MOUNT_XYZ = (0.10, 0.0, 0.15)      # mét
MOUNT_RPY = (0.0, 0.0, 0.0)        # radian
```

README của bag `GO2_KHUD_16-09` có số **hiệu chuẩn thật** từ `lidar_calibrate`
trên robot, rmse 1,2 cm, overlap 0,63:

```
x: 0.24525   y: -0.03882   z: 0.10411
roll: -0.01246   pitch: 0.02703   yaw: 0.03784
```

Lệch **14,5 cm theo x**. Đây là sai số hệ thống đi thẳng vào SLAM, vì FAST-LIO
dùng extrinsic để đặt điểm vào hệ toạ độ thân. Không phải nguyên nhân chính của
12 m drift, nhưng là nhiễu nên loại bỏ **trước** khi đo drift nghiêm túc.

### 1.3 Sửa đồng hồ robot

Đo được: robot lùi **821 ngày (2,25 năm)** so với laptop.

```
laptop : 2026-09-17 11:05:59
robot  : 2024-06-18 03:40:19
System clock synchronized: no      systemd-timesyncd: inactive
```

Nguyên nhân: Go2 mất đồng bộ NTP khi không có internet, khởi động lại là lấy giờ
từ RTC sai.

**Hệ quả nếu không sửa:** bag ghi bây giờ mang dấu thời gian 2024; TF xuyên máy
không ghép được; `TIME_FROM_ROS_TIME` của driver Ouster đóng dấu sai.

```bash
ssh -t go2-eth "sudo date -s '@$(date +%s)' && date"
```

Lâu dài: bật `systemd-timesyncd`, hoặc đồng bộ từ laptop mỗi lần nối cáp.

---

## 2. Trước khi thu bag mới

### 2.1 Ghi thẳng từ driver Ouster, đừng qua bridge

Hai đường ghi cho ra dữ liệu **khác nhau về chất**:

| ghi bằng | topic | frame | fields | byte/điểm |
|---|---|---|---|---|
| `./run.sh rec` (driver `ouster_ros`) | `/ouster/points` | `os_lidar` | x,y,z,intensity,**t**,reflectivity,ring,ambient,range | 48 |
| bridge `go2_perception` | `/go2/ouster/points` | `os_sensor` | x,y,z,intensity,ring | 20 |

Bridge làm rơi mất trường `t` — thứ FAST-LIO cần để khử méo chuyển động. Đây là
lý do phải có `go2_bridge_ouster.yaml` riêng, và cũng là nghi phạm của 12 m drift.

### 2.2 Thu bag đúng cách

- **Đứng yên 3–5 giây lúc bắt đầu.** FAST-LIO cần khoảng tĩnh để ước lượng trọng
  lực và bias IMU. Bag `26_vatdong` thiếu đúng chỗ này: |gyro| trung bình 0,762
  rad/s ngay từ message đầu.
- **Đi vòng khép kín, kết thúc đúng điểm xuất phát.** Dán băng dính đánh dấu sàn.
  Không có vòng khép kín thì không đo được drift và loop closure không có gì để bắt.
- **Ghi ≥5 phút**, vài tốc độ khác nhau, có cả đoạn xoay tại chỗ.
- **Thêm `/lowstate`** nếu sau này muốn mô hình robot có chân cử động. Không bag
  nào hiện có chứa dữ liệu góc khớp.

### 2.3 Dung lượng

`./run.sh rec` ghi khoảng **0,94 GB mỗi phút**. Robot còn 377 GB.

---

## 3. Lộ trình lifelong SLAM

Thứ tự phụ thuộc, mỗi giai đoạn có điều kiện kết thúc đo được.

**GĐ 1 — bag khép kín.** ✔ `GO2_KHUD_16-09/khuD_16-09_mau` (371 s, ~1,25 vòng)
đã đạt. Nhưng nó qua bridge nên thiếu `time`; nên thu lại một bag tương đương
bằng `./run.sh rec`.

**GĐ 2 — thước đo drift.** Đang dở. Đã đo thủ công được 12,33 m / 181 m; cần viết
thành script trong `scripts/` để chạy lặp lại và so sánh giữa các lần chỉnh.
*Xong khi:* một lệnh, đầu vào là bag, đầu ra là drift tính bằng mét.

**GĐ 3 — loop closure + pose graph.** Khoảng cách lớn nhất. Giữ FAST-LIO làm
odometry, thêm node Scan Context + GTSAM. Ứng viên: `FAST_LIO_SLAM_ros2` (khai
báo chạy từ Foxy, phụ thuộc `livox_ros_driver2` đã có sẵn).
*Xong khi:* drift của GĐ 2 giảm ít nhất một bậc độ lớn sau khi khép vòng.
**Đừng bắt đầu khi drift còn 6,8%** — quá lớn để pose graph gánh.

**GĐ 4 — bản đồ nạp lại được + định vị lại.** Lưu pose graph + keyframe +
descriptor, không phải chỉ `scans.pcd`. Thêm chế độ chỉ định vị.
*Xong khi:* bật robot ở chỗ bất kỳ trong phòng đã map, nó tự tìm ra vị trí.

**GĐ 5 — đa phiên + bảo trì bản đồ.** Gộp phiên, cắt tỉa điểm dư, loại vật thể
động, phát hiện thay đổi.
*Xong khi:* phiên ngày 2 khớp vào bản đồ ngày 1; kích thước bản đồ thôi tăng
tuyến tính theo thời gian.

**GĐ 6 — lên robot.** ✔ Đã đo: Jetson chạy **10,1 scan/giây**, load 0,72. Có dư
địa tính toán cho GĐ 3.

---

## 4. Quyết định còn treo

**Foxy hay Humble.** Foxy hết vòng đời từ 5/2023; phần lớn công cụ lifelong SLAM
nhắm Humble trở lên. Docker đã tách distro khỏi OS của robot — chính nó cho phép
chạy Humble ngay trên Jetson. Chi phí đổi trả một lần ở GĐ 3; chi phí ở lại trả
dần suốt GĐ 4 và 5.

**Camera RGB.** Robot đã có sẵn RealSense D435i (`librealsense2 2.54`,
`pyrealsense2` đã cài). Tô màu point cloud **không sửa được drift** và còn che
giấu lỗi hình học. Chỉ đáng làm khi (a) dùng cho loop closure bằng thị giác ở
GĐ 3, hoặc (b) cần bản đồ đẹp để báo cáo — và làm **sau** khi hình học đã đúng.

---

## 5. Bẫy đã biết — đọc trước khi debug

Những lỗi đã tốn thời gian, ghi lại để không mất lần hai.

**`ROS_DOMAIN_ID=0` làm DDS sập trên Go2.** Xác minh bằng gdb: SIGSEGV trong
`ddsi_plist_init_frommsg()` của `libddsc.so.0`, luồng `dq.builtins` — hàm phân
tích gói discovery. Domain 0 → sập ngay; domain 42 và 77 → chạy bình thường.
`ros2 topic list` trên domain 0 cũng chết với `bad_alloc caught` liên tục.
`docker-compose.go2.yml` dùng `GO2_SLAM_DOMAIN_ID` (**không** phải `ROS_DOMAIN_ID`,
vì `~/.bashrc` của robot đã `export ROS_DOMAIN_ID=0` và cú pháp `:-` chỉ dùng mặc
định khi biến chưa đặt).

**Lưu bản đồ là mã chết ở upstream.** Khối tích luỹ điểm trong
`publish_frame_world()` bị comment, nên `pcd_save_en: true` im lặng không lưu gì.
`patches/0002` bỏ comment. Thiếu patch là Ctrl-C xong không có file nào.

**Bag qua bridge cần extrinsic đồng nhất.** Điểm đã ở hệ `os_sensor`; áp thêm
`diag(-1,-1,1)` của bag driver là sai 180°. Triệu chứng: mô hình robot quay ngược
chiều đi (đo được: 63% số đoạn lệch 150–180° so với vector vận tốc).

**`run.sh stop` từng không giết được `ros2 bag play`** (pkill chuỗi `rosbag`,
nhưng dòng lệnh là `bag play`). Đếm được 7 bản sao cùng phát một bag → FAST-LIO
nhận dữ liệu mâu thuẫn và **phân kỳ**, vị trí nhảy lên 8×10⁸ mét. Đã sửa. Nếu
thấy kết quả vô lý, **đếm tiến trình trước khi nghi thuật toán**.

**`pkill -f` tự giết chính nó.** Dòng lệnh của shell đang chạy `pkill` cũng chứa
chuỗi tìm kiếm. Dùng `pkill -f "[o]s_driver"`, và đừng đặt tên file cần xoá trong
cùng một lệnh với `pkill`.

**rviz là nút thắt, không phải FAST-LIO.** Đo được: không rviz 9,1 scan/s; có
rviz 3,1 scan/s. Mọi phép đo hiệu năng phải chạy `rviz:=false`.

**Multicast bị chặn trên hotspot điện thoại.** Gửi 20 gói, nhận 0. DDS discovery
chết hoàn toàn và im lặng. Dùng `DDS_PEERS=ip1,ip2` để đi unicast. Băng thông
hotspot đo được 3 Mbit/s với 4% mất gói — point cloud cần 58 Mbit/s và bị cắt
thành ~186 mảnh, tỉ lệ tới nguyên vẹn 0,96^186 ≈ 0,05%.

**Dán nhiều dòng vào terminal làm mất lệnh.** `./run.sh shell` mất một hai giây
để mở; các dòng dán sau bị terminal host nuốt. Gõ từng dòng.
