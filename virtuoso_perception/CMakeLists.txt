cmake_minimum_required(VERSION 3.8)
project(virtuoso_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(virtuoso_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)  #指定pcl1.8路径

include_directories(${PCL_INCLUDE_DIRS})

add_executable(euclidean_clustering src/euclidean_clustering_node.cpp)
ament_target_dependencies(euclidean_clustering rclcpp sensor_msgs geometry_msgs visualization_msgs virtuoso_msgs pcl_conversions PCL)
target_link_libraries(euclidean_clustering ${PCL_LIBRARIES})

add_executable(ground_filter src/ground_filter.cpp)
ament_target_dependencies(ground_filter rclcpp tf2_ros pcl_ros sensor_msgs pcl_conversions PCL)
target_link_libraries(ground_filter ${PCL_LIBRARIES})


install(TARGETS
  euclidean_clustering
  ground_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/buoys/buoy_cam_filter_node.py
  scripts/buoys/buoy_lidar_node.py
  scripts/buoys/channel_node.py
  scripts/buoys/classify_buoys_node.py
  scripts/camera_processing/noise_filter_node.py
  scripts/camera_processing/resize_node.py
  scripts/code/scan_code_node.py
  scripts/dock/find_dock_codes_node.py
  scripts/dock/find_dock_entrances_node.py
  scripts/dock/find_dock_posts_node.py
  scripts/lidar_processing/self_filter_node.py
  scripts/lidar_processing/shore_filter_node.py
  scripts/stereo/buoy_stereo_node.py
  scripts/stereo/dock_stereo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
