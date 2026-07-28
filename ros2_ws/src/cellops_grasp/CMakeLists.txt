cmake_minimum_required(VERSION 3.8)
project(cellops_grasp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # benchmark numbers are meaningless in Debug
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cellops_interfaces REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# core estimator library (no ROS dependency — reused by node and benchmark)
add_library(cube_pose src/cube_pose.cpp)
target_include_directories(cube_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cube_pose Eigen3::Eigen)

add_executable(grasp_service_node src/grasp_service_node.cpp)
target_link_libraries(grasp_service_node cube_pose)
ament_target_dependencies(grasp_service_node
  rclcpp sensor_msgs geometry_msgs cellops_interfaces)

add_executable(pose_benchmark src/benchmark_main.cpp)
target_link_libraries(pose_benchmark cube_pose)

install(TARGETS grasp_service_node pose_benchmark
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
