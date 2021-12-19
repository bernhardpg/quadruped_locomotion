cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
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

# Ros nodes
add_executable(controller_node src/control/controller_node.cpp src/control/controller.cpp)
target_link_libraries(controller_node drake::drake)
target_link_libraries(controller_node ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(controller_node Eigen3::Eigen)

# Dynamics library
add_executable(dynamics src/dynamics.cpp)
target_link_libraries(dynamics Eigen3::Eigen)
target_link_libraries(dynamics pinocchio::pinocchio)

# Motion planner
add_library(planner src/planner/planner.cpp)
target_link_libraries(planner Eigen3::Eigen)
target_link_libraries(planner drake::drake)

add_executable(planner_test src/planner/planner_run.cpp)
target_link_libraries(planner_test drake::drake)




