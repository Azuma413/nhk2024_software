cmake_minimum_required(VERSION 3.8)
project(actuator_status_msg)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThreeBools.msg"
)
ament_auto_package()