cmake_minimum_required(VERSION 3.22)
project(mycobot_moveit)

find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mycobot_mtc_tutorial src/mycobot_mtc_tutorial.cpp)
ament_target_dependencies(mycobot_mtc_tutorial rclcpp moveit_task_constructor_core)
target_include_directories(mycobot_mtc_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mycobot_mtc_tutorial PUBLIC c_std_99 cxx_std_17)

install(TARGETS mycobot_mtc_tutorial DESTINATION lib/${PROJECT_NAME})

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
