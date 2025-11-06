cmake_minimum_required(VERSION 3.8)
project(robot_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  elephant_robot/robot_interface_node.py
  scripts/robot_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(FILES
  elephant_robot/robot_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(FILES
  launch/robot_interface.launch.py
  DESTINATION share/${PROJECT_NAME}/
)

# Install YAML config
install(FILES
  config/robot_config.yaml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
