cmake_minimum_required(VERSION 2.8.3)
project(vn300)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH true)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	roscpp
	message_generation
	std_msgs
)

## Necessary for finding libvnprog
find_package(PkgConfig REQUIRED)
pkg_check_modules(VNPROG REQUIRED libvnproglib-cpp)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files( FILES
	Pose.msg
	Velocities.msg
	Status.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
	std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include 
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS} 
	${VNPROG_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(vn300_node src/gps_node.cpp)

add_dependencies(vn300_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries( vn300_node
	${catkin_LIBRARIES}
	${VNPROG_LIBRARIES}
)
