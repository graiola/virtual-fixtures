cmake_minimum_required(VERSION 2.8.3)
project(vf_gui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(Boost COMPONENTS system REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp roslib actionlib
                                        actionlib_msgs mechanism_manager)

# Find the QtWidgets library
find_package(Qt5Widgets)

# Find Qt DBus
find_package(Qt5DBus)

# Catkin specific configuration
catkin_package(
 #CATKIN_DEPENDS mechanism_manager
 #DEPENDS system_lib boost
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Parse the ui files
qt5_wrap_ui(ui_mainwindow.h forms/mainwindow.ui)

# Tell CMake to create the executable
add_executable(vf_gui ui_mainwindow.h include/mainwindow.h src/mainwindow.cpp src/main.cpp)

# Use the Widgets and DBus modules from Qt 5.
target_link_libraries(vf_gui Qt5::Widgets Qt5::DBus ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(
  vf_gui
  ${mechanism_manager_EXPORTED_TARGETS}
)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} vf_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
