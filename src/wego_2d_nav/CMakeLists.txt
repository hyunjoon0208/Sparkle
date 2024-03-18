cmake_minimum_required(VERSION 3.0.2)
project(wego_2d_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wego_2d_nav
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/cmd_vel_to_ackermann.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


