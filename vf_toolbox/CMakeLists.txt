cmake_minimum_required(VERSION 2.8.3)
project(vf_toolbox)

find_package(catkin REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy roslib std_msgs realtime_tools
)

## Visualize the folders src and include in QtCreator
FILE(GLOB_RECURSE include_files "include/*.h")
add_custom_target(headers SOURCES ${include_files})
