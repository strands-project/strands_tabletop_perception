cmake_minimum_required(VERSION 2.8.3)
project(singleview_object_recognizer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp rospy recognition_srv_definitions
                                        strands_v4r genmsg actionlib_msgs actionlib
                                        scitos_ptu mongodb_store_msgs mongodb_store)

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV 2.4 REQUIRED )

#list(${FAAT_PCL_LIBRARIES} APPEND boost_regex)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(DIRECTORY action FILES CheckObjectPresence.action)
generate_messages(DEPENDENCIES actionlib_msgs)

## Generate services in the 'srv' folder
#add_service_files(
# FILES
#  recognize.srv
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  std_msgs
# geometry_msgs
#)

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
   CATKIN_DEPENDS roscpp rospy recognition_srv_definitions strands_v4r scitos_ptu mongodb_store_msgs mongodb_store
)

###########
## Build ##
###########

add_definitions(-std=c++0x)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(incs include/pcl_conversions.h include/segmenter.h)

## Declare a cpp executable
add_executable(recognition_service src/recognition_service.cpp ${incs})
add_executable(checkObjectPresence src/CheckObjectPresence.cpp)
add_executable(checkObjectPresence_client src/CheckObjectPresence_client.cpp)
add_executable(ply_to_pcl src/ply_to_pcl.cpp)
add_executable(test_recognition_service src/test_recognition_service.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(recognition_service ${catkin_EXPORTED_TARGETS} ${STRANDSV4R_LIBS})
add_dependencies(checkObjectPresence ${catkin_EXPORTED_TARGETS} ${STRANDSV4R_LIBS})
add_dependencies(checkObjectPresence_client ${catkin_EXPORTED_TARGETS})
add_dependencies(ply_to_pcl ${catkin_EXPORTED_TARGETS})
add_dependencies(test_recognition_service ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(recognition_service
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${STRANDSV4R_LIBS}
)

target_link_libraries(checkObjectPresence
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

target_link_libraries(checkObjectPresence_client
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

target_link_libraries(test_recognition_service
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${STRANDSV4R_LIBS}
)

target_link_libraries(ply_to_pcl
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

#############
## Install ##
#############

install(PROGRAMS
scripts/action_server.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS recognition_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
FILES_MATCHING PATTERN "*.launch"
)																																																																																																																

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
