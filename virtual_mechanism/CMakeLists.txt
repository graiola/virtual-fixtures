cmake_minimum_required(VERSION 2.8.3)
project(virtual_mechanism)

#set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  toolbox
)

find_package(Boost COMPONENTS filesystem system serialization REQUIRED) 

set(DMP_LIBRARIES dmp dynamicalsystems functionapproximators)

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
 CATKIN_DEPENDS roscpp rospy std_msgs toolbox
 DEPENDS system_lib boost dmpbbo
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
    include/${PROJECT_NAME}/virtual_mechanism_interface.h
    include/${PROJECT_NAME}/virtual_mechanism_line.h
    include/${PROJECT_NAME}/virtual_mechanism_dmp.h
    include/${PROJECT_NAME}/virtual_mechanism_gmr.h
    src/virtual_mechanism_line.cpp
    src/virtual_mechanism_dmp.cpp
    src/virtual_mechanism_gmr.cpp
)

## Declare a cpp executable
# add_executable(virtual_mechanism_node src/virtual_mechanism_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(virtual_mechanism_node virtual_mechanism_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${DMP_LIBRARIES} ${Boost_LIBRARIES})

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
install(TARGETS virtual_mechanism
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
catkin_add_gtest(test_virtual_mechanism_gmr 
  test/test_virtual_mechanism_gmr.cpp
)
if(TARGET test_virtual_mechanism_gmr)
  target_link_libraries(test_virtual_mechanism_gmr ${PROJECT_NAME})
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
