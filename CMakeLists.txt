cmake_minimum_required(VERSION 3.8)
project(hesai_to_velodyne)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

add_executable(hesai_to_velodyne_node src/hesai_to_velodyne_node.cpp)

target_include_directories(hesai_to_velodyne_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(hesai_to_velodyne_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(hesai_to_velodyne_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

install(TARGETS hesai_to_velodyne_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
