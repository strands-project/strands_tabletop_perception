cmake_minimum_required(VERSION 2.8.3)
project(strands_perception_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  FILES
  Table.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
)

