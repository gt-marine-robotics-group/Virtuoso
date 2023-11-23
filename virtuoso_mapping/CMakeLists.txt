cmake_minimum_required(VERSION 3.8)
project(virtuoso_mapping)

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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)  #指定pcl1.8路径

include_directories(${PCL_INCLUDE_DIRS})

add_executable(pcl_to_map_frame virtuoso_mapping/pcl_to_map_frame_node.cpp)
ament_target_dependencies(pcl_to_map_frame rclcpp sensor_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs pcl_conversions PCL)
target_link_libraries(pcl_to_map_frame ${PCL_LIBRARIES})

install(TARGETS
  pcl_to_map_frame
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  virtuoso_mapping/occupancy_map_generator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
