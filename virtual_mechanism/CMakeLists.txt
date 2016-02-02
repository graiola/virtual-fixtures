cmake_minimum_required(VERSION 2.8.3)
project(virtual_mechanism)

set(CMAKE_CXX_FLAGS "-std=c++0x  ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -DEIGEN_MALLOC_CHECKS ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${CMAKE_CXX_FLAGS_RELEASE}")

find_package(Boost COMPONENTS filesystem system serialization REQUIRED) 
set(DMP_LIBRARIES dmp dynamicalsystems functionapproximators)

if(NO_ROS_LIBS)

	set(INCLUDE_INSTALL_DIR ${PROJECT_INCLUDE_DIR}/${PROJECT_NAME})
	set(INCLUDE_PATHS ${Boost_INCLUDE_DIR} ${TOOLBOX_INCLUDE_DIR})
	set(LINK_LIBS ${DMP_LIBRARIES} ${Boost_LIBRARIES})
	set(ARCHIVE_DESTINATION ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})
	set(LIBRARY_DESTINATION ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})
	set(RUNTIME_DESTINATION ${PROJECT_RUNTIME_OUTPUT_DIRECTORY})

        ## Add gtest based cpp test target and link libraries
        enable_testing()
        find_package(GTest)
        if(GTEST_FOUND)
            include_directories(${GTEST_INCLUDE_DIRS})
            add_executable(test_virtual_mechanism_gmr
                test/test_virtual_mechanism_gmr.cpp)
            target_link_libraries(test_virtual_mechanism_gmr ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME})
        endif(GTEST_FOUND)

else()
	## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
	  toolbox
	)

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
	 CATKIN_DEPENDS toolbox
	 DEPENDS system_lib boost dmpbbo
	)

	set(INCLUDE_INSTALL_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
	set(INCLUDE_PATHS ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${TOOLBOX_INCLUDE_DIR})
	set(LINK_LIBS ${catkin_LIBRARIES} ${DMP_LIBRARIES} ${Boost_LIBRARIES})
	set(ARCHIVE_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
	set(LIBRARY_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
	set(RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

	add_definitions(-DINCLUDE_ROS_CODE)

	## Add gtest based cpp test target and link libraries
        catkin_add_gtest(test_virtual_mechanism_gmr
          test/test_virtual_mechanism_gmr.cpp
	)
        if(TARGET test_virtual_mechanism_gmr)
          target_link_libraries(test_virtual_mechanism_gmr ${PROJECT_NAME})
	endif()

endif(NO_ROS_LIBS)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${INCLUDE_PATHS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SHARED_OR_STATIC}
    include/${PROJECT_NAME}/virtual_mechanism_interface.h
    include/${PROJECT_NAME}/virtual_mechanism_gmr.h
    src/virtual_mechanism_gmr.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} test_virtual_mechanism_gmr
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
