cmake_minimum_required(VERSION 3.8)
project(racing_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node rclcpp std_msgs nav_msgs ackermann_msgs)

add_executable(twist_to_ackermann src/twist_to_ackermann.cpp)
ament_target_dependencies(twist_to_ackermann rclcpp std_msgs nav_msgs ackermann_msgs)

add_executable(joint_odometry_node src/joint_odom.cpp)
ament_target_dependencies(joint_odometry_node rclcpp sensor_msgs nav_msgs)

install(TARGETS 
  odometry_node 
  twist_to_ackermann 
  joint_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts/drift_evaluator.py
  webots_supervisor/ground_truth_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config webots_supervisor
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
