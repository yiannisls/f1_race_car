cmake_minimum_required(VERSION 3.8)
project(racer_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(obstacle_clustering src/experimental/obstacle_clustering.cpp)
target_link_libraries(obstacle_clustering ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_clustering rclcpp sensor_msgs visualization_msgs pcl_conversions)

add_executable(camera_rgb_wall_filter src/experimental/camera_rbg_wall_filter.cpp)
target_link_libraries(camera_rgb_wall_filter ${OpenCV_LIBRARIES})
ament_target_dependencies(camera_rgb_wall_filter rclcpp sensor_msgs cv_bridge message_filters)

install(TARGETS
  obstacle_clustering
  camera_rgb_wall_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/scripts/mppi_car.py
  src/scripts/global_mppi.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
  install(DIRECTORY data
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()