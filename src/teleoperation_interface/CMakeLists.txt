cmake_minimum_required(VERSION 3.8)
project(teleoperation_interface)

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

# Install Python modules (scripts are installed as PROGRAMS below)

# Install Python scripts
install(PROGRAMS
  vr_teleoperation/vr_data_server.py
  vr_teleoperation/vr_teleoperation_node.py
  scripts/vr_teleoperation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(FILES
  vr_teleoperation/vr_teleoperation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install YAML config files
# 注意：不再需要 vr_data.json，现在使用ROS2话题传输数据
install(FILES
  config/teleoperation_config.yaml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
