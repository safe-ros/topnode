cmake_minimum_required(VERSION 3.8)
project(topnode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mcap_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(topnode_interfaces REQUIRED)


include_directories(include)

add_library(resource_monitor_node SHARED
  src/resource_info.cpp
  src/resource_monitor_node.cpp
  src/mcap_writer.cpp
  )
target_compile_features(resource_monitor_node PUBLIC cxx_std_17)
target_compile_definitions(resource_monitor_node PRIVATE "TOPNODE_DLL")
ament_target_dependencies(resource_monitor_node
  rclcpp rclcpp_components rclcpp_lifecycle topnode_interfaces mcap_vendor)

add_executable(resource_monitor src/topnode.cpp)
target_compile_features(resource_monitor PUBLIC cxx_std_17)
target_link_libraries(resource_monitor resource_monitor_node)
ament_target_dependencies(resource_monitor rclcpp)

rclcpp_components_register_nodes(resource_monitor_node
  "ResourceMonitorNode"
)

install(TARGETS
  resource_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  resource_monitor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
