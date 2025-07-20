cmake_minimum_required(VERSION 3.8)
project(static_tf_republisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

add_library(static_tf_republisher_lib
  src/static_tf_republisher.cpp
)

ament_target_dependencies(static_tf_republisher_lib
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  yaml_cpp_vendor
)

add_executable(static_tf_republisher_node src/static_tf_republisher_node.cpp)
target_link_libraries(static_tf_republisher_node static_tf_republisher_lib)

ament_target_dependencies(static_tf_republisher_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  yaml_cpp_vendor
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cppcheck()
  ament_cpplint()
endif()

install(TARGETS
  static_tf_republisher_lib
  static_tf_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
