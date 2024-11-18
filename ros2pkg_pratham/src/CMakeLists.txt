cmake_minimum_required(VERSION 3.5)
project(gps_to_global_coordinates)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files (if you have any launch files in the 'launch' directory)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

