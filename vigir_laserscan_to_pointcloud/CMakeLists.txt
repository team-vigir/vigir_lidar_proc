cmake_minimum_required(VERSION 2.8.3)
project(vigir_laserscan_to_pointcloud)

## Find catkin and any catkin packages on which
## this package depends at build time

find_package(catkin REQUIRED COMPONENTS roscpp roslib laser_filters laser_geometry tf eigen_stl_containers eigen_conversions cmake_modules)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

## Declare things to be passed to other projects
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib laser_filters tf
    DEPENDS eigen 
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(laserscan_to_pointcloud_node src/laserscan_to_pointcloud_node.cpp)
target_link_libraries(laserscan_to_pointcloud_node ${catkin_LIBRARIES})
add_dependencies(laserscan_to_pointcloud_node ${catkin_EXPORTED_TARGETS})


#install execs
install(TARGETS
	laserscan_to_pointcloud_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the yaml files.
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml" 
)

