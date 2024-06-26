cmake_minimum_required(VERSION 3.8)
project(moiro_arm_move_group)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moiro_interfaces REQUIRED)

# add executable
add_executable(moiro_arm_move_group_topic_node src/moiro_arm_move_group_topic.cpp)
ament_target_dependencies(moiro_arm_move_group_topic_node
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  tf2_ros
  tf2_geometry_msgs
)

add_executable(moiro_arm_move_group_service_node src/moiro_arm_move_group_service.cpp)
ament_target_dependencies(moiro_arm_move_group_service_node
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  tf2_ros
  tf2_geometry_msgs
  moiro_interfaces
)

add_executable(moiro_arm_move_group_service_class_node src/moiro_arm_move_group_service_class.cpp)
ament_target_dependencies(moiro_arm_move_group_service_class_node
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  tf2_ros
  tf2_geometry_msgs
  moiro_interfaces
)


# install
install(TARGETS moiro_arm_move_group_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS moiro_arm_move_group_topic_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS moiro_arm_move_group_service_class_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY launch
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
