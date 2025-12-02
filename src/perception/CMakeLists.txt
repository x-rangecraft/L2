cmake_minimum_required(VERSION 3.8)
project(perception)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces (messages, services, actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/ObjectInfo.msg"
  "msg/SampleInfo.msg"
  "msg/GraspCandidate.msg"
  
  # Actions
  "action/Segment.action"
  "action/PointCloud.action"
  "action/Vectorize.action"
  "action/ObjectTarget.action"
  "action/ObjectRecord.action"
  "action/ObjectProcess.action"
  
  # Services - Storage
  "srv/SaveObject.srv"
  "srv/AddSample.srv"
  "srv/UpdateObject.srv"
  "srv/DeleteObject.srv"
  "srv/DeleteSample.srv"
  
  # Services - Query
  "srv/QueryByDesc.srv"
  "srv/QueryByImage.srv"
  "srv/QueryByLabel.srv"
  "srv/QueryById.srv"
  
  # Services - List
  "srv/ListObjects.srv"
  "srv/ListSamples.srv"
  "srv/ClearAll.srv"
  
  # Services - System
  "srv/GetStatus.srv"
  
  # Services - Grasp
  "srv/Grasp.srv"
  
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    builtin_interfaces
    action_msgs
)

# Install Python modules
ament_python_install_package(perception_core
  PACKAGE_DIR src/perception_core
)

# Install Python scripts
install(PROGRAMS
  scripts/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_perception tests/)
endif()

ament_package()

