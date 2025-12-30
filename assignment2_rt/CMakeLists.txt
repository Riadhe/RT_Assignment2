cmake_minimum_required(VERSION 3.8)
project(assignment2_rt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(assignment2_custom_msgs REQUIRED)

# 2. Build C++ Node (Robot Controller)
add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  assignment2_custom_msgs
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

# 3. Install C++ Node
install(TARGETS
  robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Install Python Node (UI)
install(PROGRAMS
  scripts/user_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Install Launch Files (Lecture 3 Requirement)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
