cmake_minimum_required(VERSION 3.16)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python packages
ament_python_install_package(navigation)

# Install executable wrapper
install(PROGRAMS
  bin/waypoint_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
