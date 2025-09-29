cmake_minimum_required(VERSION 3.8)
project(pick_and_place)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_python REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectsStamped.msg"
  DEPENDENCIES std_msgs
)

ament_python_install_package()
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
