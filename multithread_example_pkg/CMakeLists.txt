cmake_minimum_required(VERSION 3.8)
project(multithread_example_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(single_thread_node src/single_thread_node.cpp)
ament_target_dependencies(single_thread_node rclcpp std_msgs)

add_executable(multi_thread_node src/multi_thread_node.cpp)
ament_target_dependencies(multi_thread_node rclcpp std_msgs)

add_executable(static_thread_node src/static_thread_node.cpp)
ament_target_dependencies(static_thread_node rclcpp std_msgs)

add_executable(ex_thread_node src/ex_thread_node.cpp)
ament_target_dependencies(ex_thread_node rclcpp std_msgs)

install(TARGETS
  single_thread_node
  multi_thread_node
  static_thread_node
  ex_thread_node
  DESTINATION lib/${PROJECT_NAME})

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
