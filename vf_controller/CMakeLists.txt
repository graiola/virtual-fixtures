cmake_minimum_required(VERSION 2.8.3)
project(vf_controller)

set(CMAKE_CXX_FLAGS "-std=c++0x  ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -DEIGEN_MALLOC_CHECKS ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${CMAKE_CXX_FLAGS_RELEASE}")

find_package(catkin REQUIRED COMPONENTS
    pluginlib
    control_toolbox
    controller_interface
    forward_command_controller
    controller_manager
    hardware_interface
    roscpp
    rospy
    std_msgs
    geometry_msgs
    vf_toolbox
    message_generation
    kdl_parser
    mechanism_manager
    rostest
    )

add_message_files(
   FILES
   PoseRPY.msg
   RPY.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime mechanism_manager
    #DEPENDS system_lib
    )

include_directories(
    include ${catkin_INCLUDE_DIRS}
    )

## Declare a cpp library
add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/vf_controller.h
    include/${PROJECT_NAME}/kinematics_base_controller.h
    src/vf_controller.cpp
    src/kinematics_base_controller.cpp
    )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

##  copy python files
#install(DIRECTORY python_code
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   FILES_MATCHING PATTERN "*.py"
#   PATTERN ".svn" EXCLUDE
# )

## copy launch files
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
    )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
    config/vf_controller.yaml
    #   # myfile2
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )

install(FILES
    vf_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

#if (CATKIN_ENABLE_TESTING)
#    add_rostest_gtest(test_vf_controller
#        test/test_vf_controller.test
#        test/test_vf_controller.cpp)
#    target_link_libraries(test_vf_controller ${catkin_LIBRARIES})
#endif()
