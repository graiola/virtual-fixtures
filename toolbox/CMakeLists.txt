cmake_minimum_required(VERSION 2.8.3)
project(toolbox)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp
 roslib
 rospy
 std_msgs
 realtime_tools
)

catkin_package(
 INCLUDE_DIRS include
 #LIBRARIES toolbox
 CATKIN_DEPENDS roscpp rospy roslib std_msgs
 DEPENDS system_lib yaml-cpp
)

set(INCLUDE_INSTALL_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION}) # Has to be after catkin_package() call

if(realtime_tools_FOUND)
    message(STATUS "Use REALTIME TOOLS")
    #add_definitions(-DINCLUDE_ROS_RT_PUBLISHER)
endif(realtime_tools_FOUND)

## Visualize the folders src and include in QtCreator
FILE(GLOB_RECURSE include_files "include/*.h")
add_custom_target(headers SOURCES ${include_files})
FILE(GLOB_RECURSE src_files "src/*.cpp")
add_custom_target(src SOURCES ${src_files})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${INCLUDE_INSTALL_DIR}
)
