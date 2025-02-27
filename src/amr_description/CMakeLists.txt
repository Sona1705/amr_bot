cmake_minimum_required(VERSION 3.5)
project(amr_description)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)

# Install URDF and XACRO files properly
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

