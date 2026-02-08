cmake_minimum_required(VERSION 3.8)
project(fleetops_heartbeat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fleetops_msgs REQUIRED)

add_executable(heartbeat_publisher src/heartbeat_publisher.cpp) # Add the source file for the heartbeat node
ament_target_dependencies(heartbeat_publisher rclcpp fleetops_msgs) # Link the necessary dependencies

install(TARGETS heartbeat_publisher
  DESTINATION lib/${PROJECT_NAME}
) # Install the executable to the appropriate location
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
