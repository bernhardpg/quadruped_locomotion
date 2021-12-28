project(quadruped_locomotion)
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Boost REQUIRED COMPONENTS)

find_package(pinocchio REQUIRED COMPONENTS)

find_package(
	catkin REQUIRED COMPONENTS dynamic_reconfigure
	roscpp REQUIRED
	std_msgs REQUIRED
	gazebo REQUIRED
	Eigen3 REQUIRED
	drake CONFIG REQUIRED PATHS /opt/drake
)

generate_dynamic_reconfigure_options(
	config/DynamicControl.cfg
)

# Declare catkin package
catkin_package(
	 INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
    pinocchio
		drake
)

########
# BUILD
########

include_directories(
	SYSTEM # suppress warnings from 3rd party libs that I can't control
	${pinocchio_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${roscpp_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${drake_INCLUDE_DIRS}
	include)

# For pinocchio
# NOTE: not sure if all of these are needed
set(FLAGS ${pinocchio_CFLAGS_OTHER} -Wno-ignored-attributes -Wno-invalid-partial-specialization -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Apparently this is not the way to go
link_directories(
  ${pinocchio_LIBRARY_DIRS}
)


# Anymal gazebo plugin
add_library(anymal_plugin SHARED src/gazebo/anymal_plugin.cpp)
target_link_libraries(anymal_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

# Dynamics library
add_library(dynamics src/dynamics/dynamics.cpp)
target_link_libraries(dynamics ${pinocchio_LIBRARIES})

# Integrator library
add_library(integrator src/control/integrator.cpp)

# Controller 
add_executable(whole_body_controller src/control/whole_body_controller.cpp)
target_link_libraries(whole_body_controller drake::drake)
target_link_libraries(whole_body_controller ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(whole_body_controller dynamics)
target_link_libraries(whole_body_controller integrator)
target_link_libraries(whole_body_controller hierarchical_qp)

# Hierarchical QP
add_library(hierarchical_qp src/control/hierarchical_qp.cpp)
target_link_libraries(hierarchical_qp drake::drake)

# Joint controller
add_executable(joint_controller src/control/joint_controller.cpp)
target_link_libraries(joint_controller ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(joint_controller ${PROJECT_NAME}_gencfg)

# Joint state publisher
add_executable(joint_state_publisher src/visualize/joint_state_publisher.cpp)
target_link_libraries(joint_state_publisher ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

# Motion planner
add_library(motion_planner src/planner/motion_planner.cpp)
target_link_libraries(motion_planner drake::drake)

add_executable(motion_planner_node src/planner/motion_planner_node.cpp)
target_link_libraries(motion_planner_node ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(motion_planner_node motion_planner)
