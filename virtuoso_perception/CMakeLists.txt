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

add_executable(euclidean_clustering virtuoso_perception/lidar_processing/euclidean_clustering_node.cpp)
ament_target_dependencies(euclidean_clustering rclcpp sensor_msgs geometry_msgs visualization_msgs virtuoso_msgs pcl_conversions PCL)
target_link_libraries(euclidean_clustering ${PCL_LIBRARIES})

add_executable(ground_filter virtuoso_perception/lidar_processing/ground_filter_node.cpp)
ament_target_dependencies(ground_filter rclcpp tf2_ros pcl_ros sensor_msgs pcl_conversions PCL)
target_link_libraries(ground_filter ${PCL_LIBRARIES})

add_executable(self_filter virtuoso_perception/lidar_processing/self_filter_node.cpp)
ament_target_dependencies(self_filter rclcpp pcl_ros sensor_msgs pcl_conversions PCL)
target_link_libraries(self_filter ${PCL_LIBRARIES})

add_executable(shore_filter virtuoso_perception/lidar_processing/shore_filter_node.cpp)
ament_target_dependencies(shore_filter rclcpp pcl_ros sensor_msgs pcl_conversions PCL)
target_link_libraries(shore_filter ${PCL_LIBRARIES})

install(TARGETS
  euclidean_clustering
  ground_filter
  self_filter
  shore_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  virtuoso_perception/buoys/buoy_cam_filter_node.py
  virtuoso_perception/buoys/buoy_lidar_node.py
  virtuoso_perception/buoys/channel_node.py
  virtuoso_perception/buoys/classify_buoys_node.py
  virtuoso_perception/camera_processing/noise_filter_node.py
  virtuoso_perception/camera_processing/resize_node.py
  virtuoso_perception/code/scan_code_node.py
  virtuoso_perception/dock/find_dock_codes_node.py
  virtuoso_perception/dock/find_dock_entrances_node.py
  virtuoso_perception/dock/find_dock_posts_node.py
  # virtuoso_perception/lidar_processing/self_filter_node.py
  virtuoso_perception/lidar_processing/shore_filter_node.py
  virtuoso_perception/stereo/buoy_stereo_node.py
  virtuoso_perception/stereo/dock_stereo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
