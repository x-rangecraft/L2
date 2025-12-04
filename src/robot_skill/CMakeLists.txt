cmake_minimum_required(VERSION 3.10)
project(robot_skill)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(perception REQUIRED)

ament_python_install_package(
  robot_skill_core
  PACKAGE_DIR src/robot_skill_core
)

install(DIRECTORY config launch resource skill_sets
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/robot_skill_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_skill_node
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SkillSequence.action"
  "action/GraspRecord.action"
  "action/GraspRecordEasy.action"
  DEPENDENCIES geometry_msgs builtin_interfaces perception
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(_tests tests/test_skill_loader.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
