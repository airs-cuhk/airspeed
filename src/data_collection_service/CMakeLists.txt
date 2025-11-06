cmake_minimum_required(VERSION 3.8)
project(data_collection_service)

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

# Install Python scripts
install(PROGRAMS
  data_collection_service/data_storage_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all Python modules
install(FILES
  data_collection_service/data_subscriber.py
  data_collection_service/data_processor.py
  data_collection_service/storage_manager.py
  data_collection_service/multi_frequency_collector.py
  data_collection_service/vr_button_controller.py
  data_collection_service/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install wrapper script
install(PROGRAMS
  data_storage_node_wrapper
  DESTINATION lib/${PROJECT_NAME}
  RENAME data_storage_node
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(FILES
  config/data_collection_service_config.yaml
  config/data_storage_config.yaml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()