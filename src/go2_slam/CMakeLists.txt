cmake_minimum_required(VERSION 3.5)
project(go2_slam)

find_package(ament_cmake REQUIRED)

# Package không có code C++ — chỉ cài đặt tài nguyên.
# Nhờ --symlink-install, sửa yaml/launch có hiệu lực ngay, không cần build lại.
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
