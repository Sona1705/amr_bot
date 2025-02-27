cmake_minimum_required(VERSION 3.5)
project(amr_controller)

# Enable Python installation support
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # For Python packages
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(teleop_twist_joy REQUIRED)
find_package(joy REQUIRED)
find_package(controller_manager REQUIRED)   # For controller management
find_package(diff_drive_controller REQUIRED) # For diff drive controller

# Install Python scripts
install(
  PROGRAMS
    scripts/joystick_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  FILES
    launch/teleop_joystick.launch.py
    launch/controller_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(
  FILES config/amr_controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Correct form: no parentheses unless passing arguments
ament_package()
