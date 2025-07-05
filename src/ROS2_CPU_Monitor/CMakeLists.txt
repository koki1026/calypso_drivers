cmake_minimum_required(VERSION 3.8)
project(cpu_monitor)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Memory.msg"
)

install(PROGRAMS
  scripts/publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
