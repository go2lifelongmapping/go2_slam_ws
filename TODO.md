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
| **Drift trên vòng khép kín** | **✔ 0,11 m tại giây 334 = 0,07%** |
| Loop closure | ✘ chưa có |
| Bản đồ bền vững, định vị lại | ✘ chưa có |

---

## 1. Việc cần làm ngay

### 1.1 ~~Tách bạch nguyên nhân drift~~ — ĐÃ GIẢI QUYẾT, KHÔNG CÒN VẤN ĐỀ

**Kết luận: FAST-LIO chạy rất tốt trên bag này. Không có vấn đề drift.**

```
tại giây 334 (điểm khép vòng thật)
  quãng đường đã đi   : 167,4 m
  cách điểm xuất phát : 0,11 m
  drift tương đối     : 0,07%
```

FAST-LIO tự tìm điểm gần xuất phát nhất cũng ra đúng giây 334, cách 0,09 m —
khớp với điểm khép vòng mà README của bag nêu.

So sánh: leg odometry của robot trôi 0,26 m / 126,9 m = 0,21%. **FAST-LIO chính
xác hơn gấp ba.**

#### Vì sao trước đó tưởng là 12,33 m

Lấy **điểm cuối bag** làm điểm khép vòng. Sai: robot khép vòng ở giây 334 rồi
**đi thêm 10,4 m trong 37 giây** mới dừng record. README của bag nói rõ điều này
ở phần "Odometry của bag này RẤT chính xác", nhưng đã bị đọc sót.

**Bài học:** trước khi gọi một khoảng cách là "drift", phải xác minh hai điểm so
sánh thật sự là cùng một chỗ. Với bag đi nhiều hơn một vòng, tìm điểm quỹ đạo
**quay lại gần điểm xuất phát nhất**, đừng lấy điểm cuối.

#### Trường `time` per-point: không còn là ưu tiên

Cloud Ouster qua bridge thiếu trường `t` nên FAST-LIO không khử méo chuyển động.
Điều này vẫn **đúng về mặt kỹ thuật**, nhưng với drift 0,07% thì nó rõ ràng không
gây hại đáng kể ở tốc độ đi bộ 0,58 m/s. Ghi bag thẳng từ driver vẫn tốt hơn
(xem 2.1), nhưng không còn là việc gấp.

#### Đừng thử FAST-LIO trên L1

README của bag đã thử và thất bại: `blind` 0,35 / 0,47 và `scan_line` 1 / 2, cả
ba đều `No Effective Points`, không ra map. L1 chỉ 4.011 điểm/khung với range p50
0,45 m — quá thưa và quá gần cho FAST-LIO.

Ngoài ra `header.stamp` của L1 lệch **−239,5 giây** so với giờ ghi bag, ngay
trong cùng một bag. Ghép L1 với Ouster phải dùng **bus time**, không dùng
`header.stamp`.

### 1.2 ~~Sửa vị trí lidar~~ — ĐÃ XONG (18/09/2026)

```python
# truoc: uoc luong bang mat
MOUNT_XYZ = (0.10,    0.0,      0.15)
MOUNT_RPY = (0.0,     0.0,      0.0)

# sau: hieu chuan that, lidar_calibrate 16/09, rmse 1,2 cm
MOUNT_XYZ = (0.24525, -0.03882, 0.10411)
MOUNT_RPY = (-0.01246, 0.02703, 0.03784)
```

Static TF `body → base_link` đổi từ `-0.0976 +0.0097 -0.1575` thành
`-0.2383 +0.0592 -0.1176` m.

#### ⚠️ Việc này KHÔNG cải thiện drift — đừng kỳ vọng nhầm

`MOUNT_XYZ` / `MOUNT_RPY` chỉ đi vào **một** chỗ:

```
MOUNT_*  →  T_base_sensor  →  T_body_base  →  static TF 'body' → 'base_link'
```

**Không** được truyền vào FAST-LIO. FAST-LIO lấy extrinsic từ `extrinsic_T` /
`extrinsic_R` trong YAML — đó là phép biến đổi **lidar → IMU**, đọc từ metadata
của chính con sensor, và vốn đã đúng.

Chú thích sẵn trong `go2_slam.launch.py` đã nói rõ từ đầu: *"Sai số 1-2 cm ở
TỊNH TIẾN không ảnh hưởng chất lượng bản đồ (SLAM chỉ dùng LiDAR+IMU); nó chỉ
ảnh hưởng khi chiếu bản đồ về hệ chân robot để điều hướng."*

**Sửa nó để làm gì:** để `base_link` nằm đúng chỗ khi bạn chiếu bản đồ về hệ chân
robot — tức khi tích hợp điều hướng, đặt mô hình robot, hay tính chiều cao vật
cản so với bàn chân. Không phải để bản đồ đẹp hơn.

**Phải hiệu chuẩn lại** nếu tháo lắp lidar, đổi đế, hay vặn lại ốc.

### 1.3 ~~Sửa đồng hồ robot~~ — ĐÃ XONG (17/09/2026)

```
lech so voi laptop        : -0,106 s (phan lon la nhieu do, RTT SSH 220 ms)
RTC time                  : 2026-09-17 05:03:37 UTC = 12:03 gio VN  ✔
System clock synchronized : yes
NTP service               : active
```

RTC đã được ghi đúng nên giữ được qua khởi động lại. `date -s` chỉ đổi đồng hồ
hệ thống; không ghi RTC (`hwclock -w`) thì reboot là mất.

**Nếu tái diễn** (robot mất đồng bộ khi để lâu không mạng):

```bash
ssh -t go2-eth "sudo date -s '@$(date +%s)' && sudo hwclock -w && date"
```

Triệu chứng nhận biết: `timedatectl` báo `System clock synchronized: no`, và bag
thu ra mang mốc thời gian lùi hàng năm.

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

**GĐ 2 — thước đo drift.** Đang dở. Đã đo được **0,11 m / 167,4 m = 0,07%** trên
bag khu D. Cần viết thành script trong `scripts/` để chạy lặp lại.
*Script phải tự tìm điểm quay lại gần điểm xuất phát nhất*, đừng lấy điểm cuối —
đó chính là chỗ đã đo nhầm ra 12,33 m.
*Xong khi:* một lệnh, đầu vào là bag, đầu ra là drift tính bằng mét.

**GĐ 3 — loop closure + pose graph.** Khoảng cách lớn nhất. Giữ FAST-LIO làm
odometry, thêm node Scan Context + GTSAM. Ứng viên: `FAST_LIO_SLAM_ros2` — khai
báo "ROS ≥ Foxy" nên **chạy được trên Foxy, KHÔNG cần đổi distro trước**.
Phụ thuộc `livox_ros_driver2` đã có sẵn trong `src/`.

Làm trên Foxy trước; chỉ chuyển Humble khi gặp rào cản thật, lúc đó sẽ có lý do
cụ thể thay vì phỏng đoán.
*Xong khi:* bản đồ sau nhiều vòng không còn tường nhân đôi, và định vị lại được
trong bản đồ cũ.

**Lưu ý:** với drift 0,07% trên một vòng, loop closure KHÔNG còn là để sửa sai số
ngắn hạn — nó cần cho việc **chạy dài và đa phiên**, nơi sai số tích luỹ qua hàng
giờ, và cho **định vị lại** ở GĐ 4.

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

**Foxy hay Humble.** KHÔNG phải điều kiện tiên quyết cho GĐ 3 —
`FAST_LIO_SLAM_ros2` khai báo chạy từ Foxy. Đây là đánh đổi rủi ro, không phải
rào cản.

*Chi phí ở lại Foxy, đã trả trong thực tế:* hai lỗi CycloneDDS 0.7 gặp trong
2 ngày (domain 0 sập SIGSEGV; topic TRANSIENT_LOCAL không tới được subscriber),
cả hai đã sửa ở bản sau. Và `patches/0001` tồn tại CHỈ vì Foxy cũ — README của
nó ghi rõ `Trigger::Request::ConstSharedPtr` "mới có từ Humble". `ouster-ros`
cũng phải dùng nhánh `ros2-foxy` vì nhánh chính đòi Humble+.

*Chi phí đổi:* build lại image hai máy, dựng lại toàn bộ src/, kiểm chứng lại
mọi số đã đo. Một hai ngày, cộng rủi ro gặp lỗi mới ở chỗ đang ổn.

*Đề nghị:* làm GĐ 3 trên Foxy trước. Chuyển khi có lý do cụ thể.

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
