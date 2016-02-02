cmake_minimum_required(VERSION 2.4.6)
###############################################################################
# Project name is not mandatory, but you should use it
project(virtual-fixtures)

find_package(catkin)

if(catkin_FOUND)
	
	message(STATUS "Use ROS Libraries")
	add_definitions(-DINCLUDE_ROS_CODE)
	set(NO_ROS_LIBS false)	

	include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

	set(ROSPACK_MAKEDIST true)

	###############################################################################
	# Append to CPACK_SOURCE_IGNORE_FILES a semicolon-separated list of
	# directories (or patterns, but directories should suffice) that should
	# be excluded from the distro.  This is not the place to put things that
	# should be ignored everywhere, like "build" directories; that happens in
	# rosbuild/rosbuild.cmake.  Here should be listed packages that aren't
	# ready for inclusion in a distro.
	#
	# This list is combined with the list in rosbuild/rosbuild.cmake.  Note
	# that CMake 2.6 may be required to ensure that the two lists are combined
	# properly.  CMake 2.4 seems to have unpredictable scoping rules for such
	# variables.
	#list(APPEND CPACK_SOURCE_IGNORE_FILES /core/experimental)

	rosbuild_make_distribution(0.0.3)
	
else()

	message(STATUS "Don't use ROS Libraries")
	set(NO_ROS_LIBS true)	

        ###############################################################################
        # Set the type of libs to build
        set(SHARED_OR_STATIC "SHARED")

	###############################################################################
	# Set the PROJECT_ROOT variable
	set(PROJECT_ROOT ${PROJECT_SOURCE_DIR})

	###############################################################################
	# Set all bunch of usefull variables
	set(TOOLBOX_INCLUDE_DIR ${PROJECT_ROOT}/toolbox/include)
	set(VIRTUAL_MECHANISM_INCLUDE_DIR ${PROJECT_ROOT}/virtual_mechanism/include)
	set(PROJECT_INCLUDE_DIR ${PROJECT_ROOT}/include)
	set(PROJECT_INSTALL_BIN ${PROJECT_ROOT}/bin)
	set(PROJECT_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT}/bin)
	set(PROJECT_INSTALL_LIB ${PROJECT_ROOT}/lib)
        set(PROJECT_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_ROOT}/lib)

	###############################################################################
	# Forcing the generation dirs to be local
        #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${PROJECT_RUNTIME_OUTPUT_DIRECTORY})
        #set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBRARY_OUTPUT_DIRECTORY})

	###############################################################################
	# Add the main project src dir that contains all the others
	add_subdirectory(toolbox)
	add_subdirectory(virtual_mechanism)
	add_subdirectory(mechanism_manager)

	###############################################################################
	# Generate the setup.bash
	configure_file(setup.bash.in ${PROJECT_ROOT}/setup.bash)

endif(catkin_FOUND)
