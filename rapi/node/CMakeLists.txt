cmake_minimum_required(VERSION 3.8)
project(encoder_odometry)

# デフォルトのC++標準を設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージを探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# WiringPiライブラリを探す
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
if(NOT WIRINGPI_LIBRARIES)
  message(FATAL_ERROR "WiringPi library not found")
endif()

# 実行ファイルの追加
add_executable(encoder_odom_node src/encoder_odom_addsetting.cpp)
target_link_libraries(encoder_odom_node
  ${WIRINGPI_LIBRARIES}
)

ament_target_dependencies(encoder_odom_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# インストールルールの設定
install(TARGETS
  encoder_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# パッケージの設定を終了
ament_package()