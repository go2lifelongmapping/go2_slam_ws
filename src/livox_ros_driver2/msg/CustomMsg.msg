# Định dạng point cloud riêng của Livox.
std_msgs/Header header    # header chuẩn của ROS
uint64 timebase           # thời gian của điểm đầu tiên
uint32 point_num          # tổng số điểm
uint8  lidar_id           # id thiết bị lidar
uint8[3]  rsvd            # dự phòng
CustomPoint[] points      # dữ liệu point cloud
