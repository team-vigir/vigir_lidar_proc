cmake_minimum_required(VERSION 2.8.3)
project(vigir_lidar_proc_launch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_lidar_proc_launch
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#install the launch files.
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE  
)