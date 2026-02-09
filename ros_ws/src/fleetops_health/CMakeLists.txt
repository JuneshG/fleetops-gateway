cmake_minimum_required(VERSION 3.8)
project(fleetops_health)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fleetops_msgs REQUIRED)

add_executable(health_supervisor src/health_supervisor.cpp)
ament_target_dependencies(health_supervisor rclcpp fleetops_msgs)
target_include_directories(health_supervisor PRIVATE include)
install(TARGETS health_supervisor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_timeout test/test_timeout.cpp)
  target_include_directories(test_timeout PRIVATE include)
endif()

ament_package()
