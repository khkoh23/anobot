cmake_minimum_required(VERSION 3.8)
project(anobot_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(
  mock_pickup_bt_runner src/mock_pickup_bt_runner.cpp
)

target_include_directories(
  mock_pickup_bt_runner
  PRIVATE include
)

ament_target_dependencies(
  mock_pickup_bt_runner
  ament_index_cpp
  behaviortree_cpp
  rclcpp
  std_srvs
)

install(
  TARGETS mock_pickup_bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_trees config launch
  DESTINATION share/${PROJECT_NAME}
)

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
