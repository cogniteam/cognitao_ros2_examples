cmake_minimum_required(VERSION 3.5)
project(cognitao_ros2_examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cognitao_ros2 REQUIRED)


# Build
#
include_directories(include)

add_executable(action_server_example 
  src/server_example_main.cpp
)

ament_target_dependencies(action_server_example
  rclcpp
  cognitao_ros2
)

target_link_libraries(action_server_example 
  ${rclcpp_LIBRARIES} 
  ${cognitao_ros2_LIBRARIES} 
)

install(TARGETS action_server_example
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()




