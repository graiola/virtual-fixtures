cmake_minimum_required(VERSION 2.8.3)
project(virtual_mechanism)

set(CMAKE_CXX_FLAGS "-std=c++0x  ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -DEIGEN_MALLOC_CHECKS ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${CMAKE_CXX_FLAGS_RELEASE}")
## Set where to find the FindXXX.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake-modules")

find_package(Boost COMPONENTS filesystem system REQUIRED) 
find_package(YamlCpp REQUIRED)
#find_package(Eigen3 REQUIRED) # To find unsupported

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  toolbox
  roslib
  vf_gmr
)

find_package(realtime_tools QUIET)

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
 CATKIN_DEPENDS toolbox vf_gmr
 DEPENDS system_lib boost
)

## To find the yaml file related to the pkg
add_definitions(-DROS_PKG_NAME="${PROJECT_NAME}")

set(INCLUDE_INSTALL_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(INCLUDE_PATHS ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${TOOLBOX_INCLUDE_DIR} ${YAMLCPP_INCLUDE_DIR})
set(LINK_LIBS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAMLCPP_LIBRARY})
set(ARCHIVE_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set(LIBRARY_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set(RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(realtime_tools_FOUND)
   add_definitions(-DUSE_ROS_RT_PUBLISHER)
   message(STATUS "Realtime tools found")
   list(APPEND INCLUDE_PATHS ${realtime_tools_INCLUDE_DIRS})
   list(APPEND LINK_LIBS ${realtime_tools_LIBRARIES})
endif()

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_gmr
  test/test_virtual_mechanism_gmr.cpp
)
if(TARGET test_gmr)
  target_link_libraries(test_gmr ${PROJECT_NAME})
endif()
#catkin_add_gtest(test_spline
#  test/test_virtual_mechanism_spline.cpp
#)
#if(TARGET test_spline)
#  target_link_libraries(test_spline ${PROJECT_NAME})
#endif()
catkin_add_gtest(test_virtual_mechanism
  test/test_virtual_mechanism.cpp
)
if(TARGET test_virtual_mechanism)
  target_link_libraries(test_virtual_mechanism ${PROJECT_NAME})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${INCLUDE_PATHS} ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SHARED_OR_STATIC}
    include/${PROJECT_NAME}/virtual_mechanism_interface.h
    include/${PROJECT_NAME}/virtual_mechanism_factory.h
    include/${PROJECT_NAME}/virtual_mechanism_gmr.h
    #include/${PROJECT_NAME}/virtual_mechanism_spline.h
    src/virtual_mechanism_factory.cpp
    src/virtual_mechanism_gmr.cpp
    #src/virtual_mechanism_spline.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${INCLUDE_INSTALL_DIR}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yml"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY test/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  FILES_MATCHING PATTERN "*.txt"
)

## Visualize the folders src and include in QtCreator
FILE(GLOB_RECURSE include_files "include/*.h")
add_custom_target(headers SOURCES ${include_files})
FILE(GLOB_RECURSE src_files "src/*.cpp")
add_custom_target(src SOURCES ${src_files})
