cmake_minimum_required(VERSION 3.8)
project(maze_lidar_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the executable for the maze_lidar_sensor node with main.cpp included
add_executable(maze_lidar_sensor src/main.cpp src/maze_lidar_sensor.cpp)
ament_target_dependencies(maze_lidar_sensor rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge OpenCV tf2 tf2_ros tf2_geometry_msgs)

# Add include directories
target_include_directories(maze_lidar_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install the executable
install(TARGETS maze_lidar_sensor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
