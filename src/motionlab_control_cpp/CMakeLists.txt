cmake_minimum_required(VERSION 3.8)
project(motionlab_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_library(
  astar_core
  src/astar_core.cpp
)

target_include_directories(
  astar_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_astar_core
    test/test_astar_core.cpp
  )

  target_link_libraries(
    test_astar_core
    astar_core
  )
endif()
add_executable(go_to_goal_cpp src/go_to_goal.cpp)

ament_target_dependencies(
  go_to_goal_cpp
  rclcpp
  geometry_msgs
  turtlesim
)

install(
  TARGETS go_to_goal_cpp
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(
  waypoint_manager
  src/waypoint_manager.cpp
)

ament_target_dependencies(
  waypoint_manager
  rclcpp
  geometry_msgs
  turtlesim
)

install(
  TARGETS waypoint_manager
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(
  path_publisher
  src/path_publisher.cpp
)

ament_target_dependencies(
  path_publisher
  rclcpp
  geometry_msgs
  nav_msgs
)

install(
  TARGETS path_publisher
  DESTINATION lib/${PROJECT_NAME}

)
add_executable(
  pure_pursuit_controller
  src/pure_pursuit_controller.cpp
)

ament_target_dependencies(
  pure_pursuit_controller
  rclcpp
  geometry_msgs
  nav_msgs
  turtlesim
)

install(
  TARGETS pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(
  astar_planner
  src/astar_planner.cpp
)

ament_target_dependencies(
  astar_planner
  rclcpp
  geometry_msgs
  nav_msgs
)

target_link_libraries(
  astar_planner
  astar_core
)

install(
  TARGETS astar_planner
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(lidar_safety_stop src/lidar_safety_stop.cpp)
ament_target_dependencies(lidar_safety_stop rclcpp geometry_msgs sensor_msgs)

install(TARGETS lidar_safety_stop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
