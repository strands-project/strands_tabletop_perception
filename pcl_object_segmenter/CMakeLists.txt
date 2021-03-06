cmake_minimum_required(VERSION 2.8.3)
project(pcl_object_segmenter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS segmentation_srv_definitions roscpp rospy sensor_msgs std_msgs strands_v4r tf)
find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES grasping_recognition_master
   CATKIN_DEPENDS segmentation_srv_definitions strands_v4r
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(incs include/pcl_conversions.h)

## Declare a cpp library
# add_library(grasping_recognition_master
#   src/${PROJECT_NAME}/grasping_recognition_master.cpp
# )

## Declare a cpp executable
add_executable(pcl_object_segmenter_service src/segmenter_service.cpp ${incs})
add_executable(pcl_object_segmenter_demo src/test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(pcl_object_segmenter_service ${catkin_EXPORTED_TARGETS} ${STRANDSV4R_LIBS})
add_dependencies(pcl_object_segmenter_demo ${catkin_EXPORTED_TARGETS} ${STRANDSV4R_LIBS})

#message(STATUS "V4R_LIBRARIES:" ${V4R_LIB_DIR})
#message(STATUS "V4R_INCLUDES:" ${V4R_INCLUDE_DIR})

#link_directories(${V4R_LIB_DIR})

FIND_PACKAGE(Ceres)
IF(${Ceres_FOUND})
  INCLUDE_DIRECTORIES(${CERES_INCLUDES})
ELSE(${Ceres_FOUND})
  MESSAGE("\nATTENTION: Ceres solver is not installed!\n")
ENDIF(${Ceres_FOUND})

## Specify libraries to link a library or executable target against
target_link_libraries(pcl_object_segmenter_service
   ${catkin_LIBRARIES} 
   ${PCL_LIBRARIES}
   ${STRANDSV4R_LIBS}
   ${CERES_LIBRARIES}
)

target_link_libraries(pcl_object_segmenter_demo
   ${catkin_LIBRARIES} 
   ${PCL_LIBRARIES}
    ${STRANDSV4R_LIBS}
   ${CERES_LIBRARIES}

)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS grasping_recognition_master grasping_recognition_master_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grasping_recognition_master.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
