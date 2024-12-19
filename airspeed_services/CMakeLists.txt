cmake_minimum_required(VERSION 3.5)
project(airspeed_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate service msg.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DataCollection.srv"
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_package()
