cmake_minimum_required(VERSION 3.5)
project(wsg50_driver_pkg)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interface code from .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetWidth.srv"
  "srv/GetStatus.srv"
  "srv/Calibrate.srv"
  "srv/RecoverConnection.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

#Install launch files
install(DIRECTORY launch/ config/ srv/
  DESTINATION share/${PROJECT_NAME}
)

# Install resource index file
install(FILES
  package.xml
  resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python package (source dir: wsg50_driver)
ament_python_install_package(wsg50_driver)

# Export the dependencies for downstream packages
ament_export_dependencies(
  ament_cmake_python
  rosidl_default_runtime
  rclpy
  std_msgs
)

ament_package()
