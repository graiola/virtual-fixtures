cmake_minimum_required(VERSION 2.8.3)
project(mechanism_manager)

#set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(catkin 
  REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  toolbox
  virtual_mechanism
  COMPONENTS
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  realtime_tools
)

if(realtime_tools_FOUND AND sensor_msgs_FOUND)
   add_definitions(-DUSE_ROS_RT_PUBLISHER)
   message(STATUS "Realtime tools found")
endif()

find_package(Boost COMPONENTS filesystem system serialization REQUIRED) 

set(DMP_LIBRARIES dmp dynamicalsystems functionapproximators)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib rospy std_msgs toolbox virtual_mechanism
  DEPENDS system_lib boost dmpbbo yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME} 
    include/${PROJECT_NAME}/mechanism_manager.h
    src/mechanism_manager.cpp
)

## Declare a cpp executable
# add_executable(mechanism_manager_node src/mechanism_manager_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mechanism_manager_node mechanism_manager_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${DMP_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
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
#catkin_add_gtest(test_virtual_mechanism_line 
#  test/test_virtual_mechanism_line.cpp
#)
#if(TARGET test_virtual_mechanism_line )
#  target_link_libraries(test_virtual_mechanism_line  ${PROJECT_NAME})
#endif()

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_mechanism_manager 
  test/test_mechanism_manager.cpp
)
if(TARGET test_mechanism_manager)
  target_link_libraries(test_mechanism_manager ${PROJECT_NAME})
endif()

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(test_virtual_mechanism_dmp 
#  test/test_virtual_mechanism_dmp.cpp
#)
#if(TARGET test_virtual_mechanism_dmp)
#  target_link_libraries(test_virtual_mechanism_dmp ${PROJECT_NAME})
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
