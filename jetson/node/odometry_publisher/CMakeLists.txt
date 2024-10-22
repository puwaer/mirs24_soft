cmake_minimum_required(VERSION 3.5)
project(odometry_publisher)

# C++14の指定
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2の依存パッケージを探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# ノードのビルド設定
add_executable(odometry_publisher src/odometry_publisher.cpp)

# リンカにROS 2ライブラリをリンク
ament_target_dependencies(odometry_publisher 
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# インストール設定
install(TARGETS
  odometry_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# パッケージのエクスポート設定
ament_package()
