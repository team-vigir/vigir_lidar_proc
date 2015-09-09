cmake_minimum_required(VERSION 2.8.3)
project(vigir_cloud_proximity_warner)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_cloud_to_planar_scan
#  CATKIN_DEPENDS laser_geometry roscpp sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cloud_proximity_warn_node src/cloud_proximity_warn_node.cpp)

target_link_libraries(cloud_proximity_warn_node
  ${catkin_LIBRARIES}
)

install(TARGETS cloud_proximity_warn_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
