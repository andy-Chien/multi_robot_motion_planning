cmake_minimum_required(VERSION 3.8)
project(mr_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
	DIRECTORY config launch rviz srdf
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
