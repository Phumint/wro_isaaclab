cmake_minimum_required(VERSION 3.8)
project(my_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(ackermann_steering_controller REQUIRED)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  gazebo_ros
  robot_state_publisher
  urdf
  xacro
  ros_gz_sim
  ros_gz_bridge
  controller_manager
  joint_state_broadcaster
  ackermann_steering_controller
  gazebo_ros2_control
)

ament_package()
