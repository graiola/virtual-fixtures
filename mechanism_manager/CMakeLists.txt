cmake_minimum_required(VERSION 2.8.6)
project(mechanism_manager)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -DEIGEN_MALLOC_CHECKS ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${CMAKE_CXX_FLAGS_RELEASE}")

find_package(Boost COMPONENTS filesystem system serialization REQUIRED) 
set(DMP_LIBRARIES dmp dynamicalsystems functionapproximators)


if(TEST_ENABLED)
    set(BIN_TEST_NAME test_${PROJECT_NAME})
endif(TEST_ENABLED)

if(NO_ROS_LIBS)
	set(INCLUDE_INSTALL_DIR ${PROJECT_INCLUDE_DIR}/${PROJECT_NAME})
	set(INCLUDE_PATHS ${Boost_INCLUDE_DIR} ${TOOLBOX_INCLUDE_DIR} ${VIRTUAL_MECHANISM_INCLUDE_DIR})
        set(LINK_LIBS virtual_mechanism ${DMP_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
	set(ARCHIVE_DESTINATION ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})
	set(LIBRARY_DESTINATION ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})
	set(RUNTIME_DESTINATION ${PROJECT_RUNTIME_OUTPUT_DIRECTORY})

	add_definitions(-DPATH_TO_PKG="${PROJECT_ROOT}/${PROJECT_NAME}")

        ## Add gtest based cpp test target and link libraries
        enable_testing()
        find_package(GTest)
        if(GTEST_FOUND AND TEST_ENABLED)
            include_directories(${GTEST_INCLUDE_DIRS})
            add_executable(${BIN_TEST_NAME}
              test/test_mechanism_manager.cpp)
            target_link_libraries(${BIN_TEST_NAME} ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME})
        endif(GTEST_FOUND AND TEST_ENABLED)
else()

	find_package(catkin 
	  REQUIRED COMPONENTS
	  roscpp
	  roslib
	  rospy
	  std_msgs
	  toolbox
	  virtual_mechanism
          actionlib
          actionlib_msgs
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

        add_action_files(
          DIRECTORY action
          FILES MechanismManager.action
        )

        generate_messages(
          DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
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
          CATKIN_DEPENDS actionlib_msgs
          #DEPENDS system_lib boost dmpbbo yaml-cpp
          DEPENDS boost
	)

	set(INCLUDE_INSTALL_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
	set(INCLUDE_PATHS ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${TOOLBOX_INCLUDE_DIR} ${VIRTUAL_MECHANISM_INCLUDE_DIR})
	set(LINK_LIBS ${catkin_LIBRARIES} ${DMP_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
	set(ARCHIVE_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
	set(LIBRARY_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
	set(RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

	add_definitions(-DINCLUDE_ROS_CODE)

        if(TEST_ENABLED)
            ## Add gtest based cpp test target and link libraries
            catkin_add_gtest(${BIN_TEST_NAME}
              test/test_mechanism_manager.cpp
            )
            if(TARGET ${BIN_TEST_NAME})
              target_link_libraries(${BIN_TEST_NAME} ${PROJECT_NAME})
            endif()
        endif(TEST_ENABLED)

	## Mark other files for installation (e.g. launch and bag files, etc.)
	install(DIRECTORY config/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
	  FILES_MATCHING PATTERN "*.yml"
	)   

	install(DIRECTORY models/
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
	  FILES_MATCHING PATTERN "*.txt"
	) 

        set(ALL_SRC src/mechanism_manager_server.cpp
                    include/${PROJECT_NAME}/mechanism_manager_server.h)

endif(NO_ROS_LIBS)

set(ALL_SRC ${ALL_SRC}
            src/mechanism_manager.cpp
            include/${PROJECT_NAME}/mechanism_manager.h)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${INCLUDE_PATHS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SHARED_OR_STATIC} ${ALL_SRC})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

## Advise cmake to wait for the exported targets to be compiled before processing the project target
add_dependencies(
  ${PROJECT_NAME}
  ${mechanism_manager_EXPORTED_TARGETS}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${BIN_TEST_NAME}
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

## Visualize the folders src and include in QtCreator
FILE(GLOB_RECURSE include_files "include/*.h")
add_custom_target(headers SOURCES ${include_files})
FILE(GLOB_RECURSE src_files "src/*.cpp")
add_custom_target(src SOURCES ${src_files})

