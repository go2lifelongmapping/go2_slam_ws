# Patches cho source bên thứ ba

`src/FAST_LIO/` và `src/ouster-ros/` nằm trong `.gitignore`, nên sửa đổi trong
đó KHÔNG được git theo dõi. Các patch ở đây tái tạo được sau khi clone mới.

## Áp dụng sau khi clone lại

```bash
cd ~/go2_slam_ws/src/FAST_LIO
git apply --check ../../patches/0001-fast_lio-foxy-ouster.patch   # thử trước
git apply         ../../patches/0001-fast_lio-foxy-ouster.patch
```

## 0001-fast_lio-foxy-ouster.patch

Ba sửa đổi, đều là lỗi thật gặp khi chạy FAST_LIO (nhánh ROS2) trên Foxy với
lidar Ouster. Nhánh ROS2 của FAST_LIO được viết và kiểm thử trên Humble.

### (a) `laserMapping.cpp:1115` — service callback không biên dịch được

FAST_LIO dùng `Trigger::Request::ConstSharedPtr`. rclcpp của Foxy chỉ nhận
`Request::SharedPtr` (không const); khả năng nhận const mới có từ Humble.

Triệu chứng: build fail với
`no matching function for call to 'rclcpp::AnyServiceCallback<...>::set(...)'`
và `no type named 'type' in 'struct std::enable_if<false, void>'`.

### (b) `laserMapping.cpp:929` — QoS của IMU không tương thích  ← NGHIÊM TRỌNG

```cpp
// trước:  create_subscription<Imu>(imu_topic, 10, imu_cbk)
// sau:    create_subscription<Imu>(imu_topic, rclcpp::SensorDataQoS().keep_last(200), imu_cbk)
```

Số `10` nghĩa là QoS mặc định = RELIABLE. Driver ouster_ros publish IMU ở
BEST_EFFORT (SensorDataQoS). Hai bên không tương thích nên FAST-LIO KHÔNG nhận
được mẫu IMU nào.

Triệu chứng: build sạch, node chạy, không có lỗi rõ ràng, nhưng
`/cloud_registered` im lặng ở 0 Hz. Chỉ có một dòng WARN dễ bỏ qua:
`New publisher discovered on this topic, offering incompatible QoS.`

`keep_last(200)` thay vì mặc định 5: IMU chạy 100 Hz còn FAST-LIO xử lý theo
nhịp scan 10 Hz, nên cần đệm ~10-20 mẫu mỗi vòng. Depth 5 sẽ rơi mẫu.

Dòng 927 (point cloud) đã dùng đúng `SensorDataQoS()` nên không phải sửa.

### (c) `preprocess.h:94,110` — kiểu `ring` lệch giữa driver và FAST_LIO

| | kiểu `ring` |
|---|---|
| `ouster_ros/os_point.h:27` | `uint16_t` |
| `FAST_LIO/preprocess.h:94` (gốc) | `uint8_t` |

`pcl::fromROSMsg` khớp trường theo TÊN **và** KIỂU. Lệch kiểu thì PCL bỏ qua
trường đó trong im lặng.

Triệu chứng: spam log `Failed to find match for field 'ring'.` mỗi scan.

Với `feature_extract_enable: false` (chế độ FAST-LIO2) thì `ring` không được
dùng nên không sai kết quả — nhưng sẽ sai ngay nếu bật trích xuất đặc trưng.

## 0002-fast_lio-pcd-save.patch

Bỏ comment khối tích luỹ điểm trong `publish_frame_world()`
(`laserMapping.cpp` ~dòng 516).

Upstream comment cả khối lại, nên `pcl_wait_save` không bao giờ được đổ dữ liệu
vào. Hệ quả: đặt `pcd_save_en: true` trong config KHÔNG có tác dụng — khối lưu
ở cuối `main()` kiểm tra `pcl_wait_save->size() > 0`, luôn thấy 0, rồi thoát
im lặng. Không lỗi, không cảnh báo, không file.

Service `map_save` cũng không thay thế được: nó ghi `pcl_wait_pub` (buffer của
`publish_map()`, mà ta tắt bằng `map_en: false` vì nó ngốn RAM) ra đường dẫn
tương đối `./test.pcd`.

Sau khi áp patch, Ctrl-C node sẽ ghi `src/FAST_LIO/PCD/scans.pcd`.
Đã kiểm chứng: bag 57 giây -> 8 494 164 điểm, 271 MB.

LƯU Ý BỘ NHỚ: bản đồ tích luỹ trong RAM tới lúc tắt node, khoảng **4.8 MB mỗi
giây**. Chạy 5 phút là ~1.4 GB. Với Jetson của Go2, đặt `pcd_save.interval`
(số scan) khác -1 để nó xả định kỳ ra `scans_1.pcd`, `scans_2.pcd`, ...
thay vì giữ tất cả trong bộ nhớ.
