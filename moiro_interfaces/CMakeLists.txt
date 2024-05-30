cmake_minimum_required(VERSION 3.8)
project(moiro_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/Vector2.msg"
  "msg/Pose2D.msg"
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
  "msg/Mask.msg"
  "msg/KeyPoint2D.msg"
  "msg/KeyPoint2DArray.msg"
  "msg/KeyPoint3D.msg"
  "msg/KeyPoint3DArray.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/DetectionInfo.msg"
  "msg/BBoxYolo.msg"
  "msg/FaceBox2D.msg"
  "msg/SensorState.msg"
  "srv/Person.srv"
  "srv/TargetPose.srv"
  "srv/TargetDepth.srv"
  "srv/Sound.srv"
  "action/Patrol.action"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
