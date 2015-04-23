cmake_minimum_required(VERSION 2.8.3)
project(vf_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  control_toolbox
  controller_interface
  forward_command_controller
  controller_manager
  hardware_interface
  roscpp
  rospy
  std_msgs
  kdl_kinematics
  mechanism_manager
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
 CATKIN_DEPENDS pluginlib control_toolbox controller_interface forward_command_controller controller_manager hardware_interface roscpp rospy std_msgs kdl_kinematics mechanism_manager
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} include/${PROJECT_NAME}/${PROJECT_NAME}.h src/${PROJECT_NAME}.cpp)

## Declare a cpp executable
# add_executable(controller_node src/controller_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(controller_node controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib)
#set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
#set(PKG_CONFIG_ADDITIONAL_VARIABLES
#  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
#  plugindirname
#  plugindir
#)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##  copy python files
#install(DIRECTORY python_code
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   FILES_MATCHING PATTERN "*.py"
#   PATTERN ".svn" EXCLUDE
# )

## copy launch files
install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   FILES_MATCHING PATTERN "*.launch"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  config/vf_controller.yaml
#   # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
  vf_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
