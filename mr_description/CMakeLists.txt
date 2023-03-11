cmake_minimum_required(VERSION 3.8)
project(mr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

install(
	DIRECTORY config meshes urdf world
	DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(gazebo_ros_pkgs)

ament_package()