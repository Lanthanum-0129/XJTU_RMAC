cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 查找 MVS SDK
set(MVS_INCLUDE_DIR "")
set(MVS_LIBRARY "")

if(EXISTS "/opt/MVS/include")
  set(MVS_INCLUDE_DIR "/opt/MVS/include")
  set(MVS_LIBRARY "/opt/MVS/lib/64/libMvCameraControl.so")
elseif(EXISTS "/usr/local/MVS/include")
  set(MVS_INCLUDE_DIR "/usr/local/MVS/include")
  set(MVS_LIBRARY "/usr/local/MVS/lib/64/libMvCameraControl.so")
else()
  message(FATAL_ERROR "MVS SDK not found. Please install HIKROBOT MVS SDK first.")
endif()

include_directories(
  include
  ${MVS_INCLUDE_DIR}
)

add_executable(camera_node 
  src/main.cpp 
  src/camera_node.cpp
)

ament_target_dependencies(camera_node 
  rclcpp 
  sensor_msgs 
  std_msgs
)

target_link_libraries(camera_node 
  ${MVS_LIBRARY}
)

# 安装目标
install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 和 config 文件
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()