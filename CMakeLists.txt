cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(legged_locomotion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp REQUIRED
	std_msgs REQUIRED
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(
	${roscpp_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	include)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(anymal_controller SHARED src/anymal_controller.cpp)
target_link_libraries(anymal_controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
target_link_libraries(anymal_controller Eigen3::Eigen)


