cmake_minimum_required(VERSION 3.8)
project(my_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages that are required for the build
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories to the share folder
install(
  DIRECTORY launch urdf worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Finalize the package
ament_package()
