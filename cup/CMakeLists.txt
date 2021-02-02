cmake_minimum_required(VERSION 3.5)
project(cup)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# messages
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
#find_package(resource_retriever REQUIRED)

#ament_export_dependencies(rosidl_default_runtime)

include_directories(include/${PROJECT_NAME}/)

add_executable(cup_node
        src/cup_node.cpp
        include/cup/cup_node.h)

ament_target_dependencies(cup_node
        rclcpp
        tf2_ros
        geometry_msgs
        visualization_msgs)

install(TARGETS cup_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        resource
        DESTINATION share/${PROJECT_NAME}/)


ament_package()
