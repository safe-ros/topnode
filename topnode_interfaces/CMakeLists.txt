cmake_minimum_required(VERSION 3.8)
project(topnode_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/IoStats.msg"
  "msg/LoadAvg.msg"
  "msg/MemoryState.msg"
  "msg/ProcessResourceUsage.msg"
  "msg/Stat.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
