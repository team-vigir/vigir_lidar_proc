cmake_minimum_required(VERSION 2.8.3)
project(vigir_lidar_intensity_modifier)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_lidar_intensity_modifier
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_intensity_modifier_node src/laser_intensity_modifier_node.cpp)

target_link_libraries(laser_intensity_modifier_node
   ${catkin_LIBRARIES}
)

install(TARGETS laser_intensity_modifier_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
