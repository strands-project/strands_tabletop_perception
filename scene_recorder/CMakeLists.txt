cmake_minimum_required(VERSION 2.8.3)
project(scene_recorder)
SET (CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS genmsg roscpp rospy
                                        mongodb_store_msgs mongodb_store_cpp_client
                                        actionlib_msgs actionlib
                                        geometry_msgs scitos_apps_msgs sensor_msgs tf 
                                        recognition_srv_definitions world_state table_segmentation world_state)

find_package(Boost REQUIRED COMPONENTS system thread program_options filesystem)
find_package(PCL 1.7 REQUIRED)



## Generate actions in the 'action' folder
add_action_files(DIRECTORY action FILES TakeSnapshot.action)
generate_messages(DEPENDENCIES actionlib_msgs)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy scitos_ptu mongodb_store_msgs mongodb_store_cpp_client
                geometry_msgs tf2_msgs sensor_msgs tf world_state recognition_srv_definitions scitos_apps_msgs
                table_segmentation world_state
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
set(incs include/pcl_conversions.h)


## Declare a cpp executable
#add_executable(scene_recorder_node src/scene_snapshot.cpp ${incs})

#add_dependencies(scene_recorder_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(scene_recorder_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#)

