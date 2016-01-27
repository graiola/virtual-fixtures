cmake_minimum_required(VERSION 2.8.3)
project(toolbox)

if(NO_ROS_LIBS)

	#add_custom_target(${PROJECT_NAME} ALL DEPENDS include/${PROJECT_NAME}/toolbox.h)
	#add_custom_target(${PROJECT_NAME})

	install(DIRECTORY include/${PROJECT_NAME}/
	  DESTINATION ${PROJECT_INCLUDE_DIR}/${PROJECT_NAME}
	)

else()

	add_definitions(-DINCLUDE_ROS_CODE)

	## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
	 roscpp
	 rospy
	 std_msgs
	)

	catkin_package(
	 INCLUDE_DIRS include
	 #LIBRARIES toolbox
	 CATKIN_DEPENDS roscpp rospy std_msgs
	 DEPENDS system_lib
	)

	## Mark cpp header files for installation
	install(DIRECTORY include/${PROJECT_NAME}/
	  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	)
endif(NO_ROS_LIBS)









