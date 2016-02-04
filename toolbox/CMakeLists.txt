cmake_minimum_required(VERSION 2.8.3)
project(toolbox)

if(NO_ROS_LIBS)

	set(INCLUDE_INSTALL_DIR ${PROJECT_INCLUDE_DIR}/${PROJECT_NAME})
else()

	## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
	 roscpp
	 rospy
	 std_msgs
	 realtime_tools
	)

	catkin_package(
	 INCLUDE_DIRS include
	 #LIBRARIES toolbox
	 CATKIN_DEPENDS roscpp rospy std_msgs
	 DEPENDS system_lib
	)

	set(INCLUDE_INSTALL_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION}) # Has to be after catkin_package() call

        add_definitions(-DINCLUDE_ROS_CODE)

        if(realtime_tools_FOUND)
            message(STATUS "Use REALTIME TOOLS")
            #add_definitions(-DINCLUDE_ROS_RT_PUBLISHER)
        endif(realtime_tools_FOUND)
        
	

endif(NO_ROS_LIBS)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${INCLUDE_INSTALL_DIR}
)
