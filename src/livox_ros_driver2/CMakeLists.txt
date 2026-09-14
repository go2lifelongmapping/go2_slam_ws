cmake_minimum_required(VERSION 3.5)
project(livox_ros_driver2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Sinh header C++ tại: livox_ros_driver2/msg/custom_msg.hpp
# Đúng đường dẫn mà preprocess.h:5 và laserMapping.cpp:62 include.
# Quy tắc đổi tên của rosidl: CustomMsg.msg -> custom_msg.hpp
#
# CustomPoint phải liệt kê TRƯỚC CustomMsg, vì CustomMsg dùng CustomPoint[].
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

# Bắt buộc: để FAST_LIO gọi được ament_target_dependencies(... livox_ros_driver2)
ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()
