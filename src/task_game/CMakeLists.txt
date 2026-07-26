cmake_minimum_required(VERSION 3.8)
project(task_game)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


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

add_executable(robot_control
  src/main.cpp
  src/core/robot.cpp
  src/core/pilot.cpp
  src/core/behavior_tree.cpp
  src/nodes/generate_plan.cpp
  src/nodes/arrive_to_box.cpp
  src/nodes/catch_box.cpp
  src/nodes/arrive_to_target.cpp
  src/nodes/place_box.cpp
  src/nodes/lift_search.cpp
)

add_executable(test_node
  src/test.cpp
)

add_executable(measure_node
  src/measure.cpp
)


target_include_directories(robot_control
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(
  robot_control
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  robot_msgs
)

target_link_libraries(robot_control Eigen3::Eigen yaml-cpp)

ament_target_dependencies(
  test_node
  rclcpp
  sensor_msgs
  robot_msgs
  std_msgs
)

ament_target_dependencies(
  measure_node
  ament_index_cpp
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  robot_msgs
  std_msgs
)

target_link_libraries(measure_node yaml-cpp)

install(TARGETS
robot_control
test_node
measure_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
