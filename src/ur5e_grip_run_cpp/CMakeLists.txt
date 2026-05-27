cmake_minimum_required(VERSION 3.8)
project(ur5e_grip_run_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(robotiq_2f_urcap_adapter REQUIRED)

add_executable(ik_commander
  src/joint_commander_ik.cpp
)

target_include_directories(ik_commander PUBLIC
  include
)

ament_target_dependencies(
  ik_commander
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  robotiq_2f_urcap_adapter
)

install(TARGETS
  ik_commander
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()