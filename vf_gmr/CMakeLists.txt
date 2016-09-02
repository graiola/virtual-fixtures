cmake_minimum_required(VERSION 2.8.6)
# Project name is not mandatory, but you should use it
project(vf_gmr)

# Some flags for CXX
set(CMAKE_CXX_FLAGS "-Wall -std=c++0x -fPIC ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${CMAKE_CXX_FLAGS_RELEASE}")

# Define macros to check for real time requirements
# To activate the EIGEN_MALLOC asserts use: cmake .. -DREALTIME_CHECKS=1 -DCMAKE_BUILD_TYPE=Debug
if(REALTIME_CHECKS)
   add_definitions(-DREALTIME_CHECKS)
endif()

###############################################################################
# Find boost packages
find_package( Boost COMPONENTS filesystem system REQUIRED)

###############################################################################
# Find catkin
find_package(catkin)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS 
  #DEPENDS
  DEPENDS boost
)

set(INCLUDE_INSTALL_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(INCLUDE_PATHS ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
set(LINK_LIBS ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set(ARCHIVE_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set(LIBRARY_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set(RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(GLOB SOURCES src/*.cpp) 
file(GLOB HEADERS include/${PROJECT_NAME}/*.hpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${INCLUDE_PATHS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

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
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Visualize the folders src and include in QtCreator
FILE(GLOB_RECURSE include_files "include/*.hpp")
add_custom_target(headers SOURCES ${include_files})
FILE(GLOB_RECURSE src_files "src/*.cpp")
add_custom_target(src SOURCES ${src_files})
