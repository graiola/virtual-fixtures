cmake_minimum_required(VERSION 2.8.3)
project(vf_gui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Boost COMPONENTS system REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp roslib
                                        mechanism_manager
                                        rosgraph_msgs)

# Find the QtWidgets library
find_package(Qt5Widgets)

# Catkin specific configuration
	catkin_package(
         CATKIN_DEPENDS rosgraph_msgs
	 #DEPENDS system_lib boost
	)

	## Specify additional locations of header files
	## Your package locations should be listed before other locations
	include_directories(include ${catkin_INCLUDE_DIRS})

if(Qt5Widgets_FOUND)

	message(STATUS "Qt5Widgets Found - Compile ${PROJECT_NAME}")

	# Instruct CMake to run moc automatically when needed.
	set(CMAKE_AUTOMOC ON)

	# Parse the ui files
	qt5_wrap_ui(ui_main_window.h forms/main_window.ui)

	# Tell CMake to create the executable
	add_executable(vf_gui ui_main_window.h
			      include/main_window.h
			      include/guides_model.h
			      src/main_window.cpp
			      src/guides_model.cpp
			      src/main.cpp)

	# Use the Widgets and DBus modules from Qt 5.
	target_link_libraries(vf_gui Qt5::Widgets ${Boost_LIBRARIES} ${catkin_LIBRARIES})

	add_dependencies(
	  vf_gui
	  ${mechanism_manager_EXPORTED_TARGETS}
	)

	# Mark executables and/or libraries for installation
	install(TARGETS ${PROJECT_NAME} vf_gui
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

        # Copy launch files
        install(DIRECTORY launch
           DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
           FILES_MATCHING PATTERN "*.launch"
        )

else(Qt5Widgets_FOUND)

	message(STATUS "Qt5Widgets NOT Found - DON'T Compile ${PROJECT_NAME}")

endif(Qt5Widgets_FOUND)
