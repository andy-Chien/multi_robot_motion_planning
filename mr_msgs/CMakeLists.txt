cmake_minimum_required(VERSION 3.8)
project(mr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sample.msg"
  "msg/ServoControl.msg"
  "msg/Obstacles.msg"
  "msg/Pose.msg"
  "msg/Poses.msg"
  "srv/Sample.srv"
  "srv/IK.srv"
  "srv/PathToTraj.srv"
  "srv/TrajMerge.srv"
  "srv/GetRobotTrajectoryObstacle.srv"
  "srv/SetTrajectoryState.srv"
  "action/Sample.action"
DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  moveit_msgs
  geometry_msgs
  lifecycle_msgs
  trajectory_msgs
  tf2_geometry_msgs
  controller_manager_msgs
  rosidl_default_generators
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
