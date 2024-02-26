cmake_minimum_required(VERSION 3.5)
project(counter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mrsd_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(counter_node src/counter_node.cpp)
ament_target_dependencies(counter_node rclcpp std_msgs mrsd_msgs)

install(TARGETS
  counter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()