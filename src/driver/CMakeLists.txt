cmake_minimum_required(VERSION 3.10)
project(robot_driver VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(driver PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/driver)

install(PROGRAMS
  scripts/robot_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

set(msg_files
  "msg/JointCommandTarget.msg"
  "msg/JointCommandOptions.msg"
  "msg/JointContact.msg"
)

set(action_files
  "action/JointCommand.action"
  "action/SafetyPose.action"
  "action/GripperCommand.action"
  "action/RobotCommand.action"
)

set(srv_files
  "srv/SolveIK.srv"
  "srv/SolveFK.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(FILES
  package.xml
  robot_description.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES resource/robot_driver
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
