cmake_minimum_required(VERSION 2.8.6)
project(mechanism_manager)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -DEIGEN_MALLOC_CHECKS ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${CMAKE_CXX_FLAGS_RELEASE}")
## Set where to find the FindXXX.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake-modules")

find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(RTAI)
find_package(YamlCpp REQUIRED)
#set(DMP_LIBRARIES dmp dynamicalsystems functionapproximators)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  toolbox
  vf_gmr
  virtual_mechanism
  message_generation
)

find_package(realtime_tools QUIET)

if(realtime_tools_FOUND)
   add_definitions(-DUSE_ROS_RT_PUBLISHER)
   message(STATUS "Realtime tools found")
endif()

add_service_files(FILES
  MechanismManagerServices.srv)

generate_messages()

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
  CATKIN_DEPENDS message_runtime
  #DEPENDS system_lib boost dmpbbo yaml-cpp
  DEPENDS boost
)

## To find the yaml file related to the pkg
add_definitions(-DROS_PKG_NAME="${PROJECT_NAME}")

set(INCLUDE_INSTALL_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(INCLUDE_PATHS ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${TOOLBOX_INCLUDE_DIR}
    ${VIRTUAL_MECHANISM_INCLUDE_DIR} ${RTAI_INCLUDE_DIR} ${YAMLCPP_INCLUDE_DIR})
set(LINK_LIBS ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${RTAI_LIBRARIES} ${YAMLCPP_LIBRARY})
set(ARCHIVE_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set(LIBRARY_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set(RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_mechanism_manager
  test/test_mechanism_manager.cpp
)
if(TARGET test_mechanism_manager)
  target_link_libraries(test_mechanism_manager ${PROJECT_NAME})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${INCLUDE_PATHS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/mechanism_manager_server.h
    include/${PROJECT_NAME}/mechanism_manager_interface.h
    include/${PROJECT_NAME}/mechanism_manager.h
    src/mechanism_manager_server.cpp
    src/mechanism_manager_interface.cpp
    src/mechanism_manager.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

## Advise cmake to wait for the exported targets to be compiled before processing the project target
add_dependencies(
  ${PROJECT_NAME}
  ${mechanism_manager_EXPORTED_TARGETS}
)

if(RTAI_FOUND)
    message(STATUS "RTAI found")
    add_executable(test_realtime test/test_realtime.cpp)
    target_link_libraries(test_realtime ${PROJECT_NAME})
endif()

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

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  #FILES_MATCHING PATTERN "*.txt"
)

## Visualize the folders src and include in QtCreator
FILE(GLOB_RECURSE include_files "include/*.h")
add_custom_target(headers SOURCES ${include_files})
FILE(GLOB_RECURSE src_files "src/*.cpp")
add_custom_target(src SOURCES ${src_files})
