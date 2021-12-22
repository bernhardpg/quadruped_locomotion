cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(quadruped_locomotion)

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

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Drake
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
include_directories(${drake_INCLUDE_DIRS})

# Pinoccio
find_package(pinocchio REQUIRED)

# Anymal gazebo plugin
add_library(anymal_plugin SHARED src/anymal_plugin.cpp)
target_link_libraries(anymal_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
target_link_libraries(anymal_plugin Eigen3::Eigen)

# Declare catkin package
catkin_package()

# Controller 
add_executable(controller_node src/control/controller_node.cpp src/control/controller.cpp)
target_link_libraries(controller_node drake::drake)
target_link_libraries(controller_node ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(controller_node Eigen3::Eigen)
target_link_libraries(controller_node dynamics)
target_link_libraries(controller_node pinocchio::pinocchio)

# Dynamics library
add_executable(dynamics src/dynamics/dynamics.cpp)
target_link_libraries(dynamics Eigen3::Eigen)
target_link_libraries(dynamics pinocchio::pinocchio)

# Motion planner
add_library(motion_planner src/planner/motion_planner.cpp)
target_link_libraries(motion_planner Eigen3::Eigen)
target_link_libraries(motion_planner drake::drake)

add_executable(motion_planner_node src/planner/motion_planner_node.cpp)
target_link_libraries(motion_planner_node motion_planner)
target_link_libraries(motion_planner_node ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
