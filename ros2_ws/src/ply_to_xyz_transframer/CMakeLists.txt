cmake_minimum_required(VERSION 3.8)
project(ply_to_xyz_transframer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

add_library(ply_to_xyz_transframer_lib
  src/ply_to_xyz_transframer_interface.cpp
  src/ply_to_xyz_transframer.cpp
)

ament_target_dependencies(ply_to_xyz_transframer_lib
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  std_srvs
  yaml_cpp_vendor
  pcl_conversions
)

add_executable(ply_to_xyz_transframer_interface_node src/ply_to_xyz_transframer_interface_node.cpp)
target_link_libraries(ply_to_xyz_transframer_interface_node ply_to_xyz_transframer_lib)

ament_target_dependencies(ply_to_xyz_transframer_interface_node
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  std_srvs
  yaml_cpp_vendor
  pcl_conversions
)


if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cppcheck()
  ament_cpplint()
endif()

install(TARGETS
  ply_to_xyz_transframer_lib
  ply_to_xyz_transframer_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
