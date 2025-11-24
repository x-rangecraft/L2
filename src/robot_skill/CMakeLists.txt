cmake_minimum_required(VERSION 3.10)
project(robot_skill)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(DIRECTORY config launch resource
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/robot_skill_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_skill_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_tests tests/test_placeholder.py)
endif()

ament_package()
