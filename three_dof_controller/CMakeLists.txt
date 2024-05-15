cmake_minimum_required(VERSION 3.5)
project(three_dof_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
  three_dof_controller/controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and URDF model
install(
  DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

