cmake_minimum_required(VERSION 3.8)
project(virtuoso_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(include)

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
