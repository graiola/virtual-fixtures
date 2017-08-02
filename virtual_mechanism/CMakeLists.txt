cmake_minimum_required(VERSION 2.8.3)
project(virtual_mechanism)

set(CMAKE_CXX_FLAGS "-std=c++0x  ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -DEIGEN_MALLOC_CHECKS ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${CMAKE_CXX_FLAGS_RELEASE}")

find_package(catkin REQUIRED COMPONENTS
    vf_toolbox
    roslib
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roslib
    )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/virtual_mechanism_interface.h
    include/${PROJECT_NAME}/virtual_mechanism_factory.h
    include/${PROJECT_NAME}/virtual_mechanism_spline.h
    src/virtual_mechanism_factory.cpp
    src/virtual_mechanism_spline.cpp
    )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY test/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILES_MATCHING PATTERN "*.model"
    )

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_virtual_mechanism_spline
        test/test_virtual_mechanism_spline.cpp)
    target_link_libraries(test_virtual_mechanism_spline ${PROJECT_NAME} ${catkin_LIBRARIES})
    catkin_add_gtest(test_virtual_mechanism_factory
        test/test_virtual_mechanism_factory.cpp)
    target_link_libraries(test_virtual_mechanism_factory ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

