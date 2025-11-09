cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(robot_controller)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install executable scripts manually (workaround for entry_points with symlink install)
# Create wrapper scripts for the nodes
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/motor_controller_node
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


