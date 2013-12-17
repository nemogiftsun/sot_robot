cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/base.cmake)
include(cmake/python.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/src)
set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")


set(PROJECT_DESCRIPTION "sot controller for Universal  Robot")
set(PROJECT_NAME sot_youbot)
set(PROJECT_URL "")
set(CXX_DISABLE_WERROR True)


# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mathtools")
ADD_REQUIRED_DEPENDENCY("jrl-mal")
ADD_REQUIRED_DEPENDENCY("dynamic-graph")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ADD_REQUIRED_DEPENDENCY("sot-core")

# This is required by the Python prologue.
ADD_REQUIRED_DEPENDENCY("sot-dynamic")

# Python
INCLUDE(cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

set(SOURCES
#    src/ur_sot_controller.cpp
    src/youbot_device.cpp)

set(HEADERS
#    include/ur_sot_controller.h
    include/sot_youbot/youbot_device.h)

add_library(sot_youbot ${HEADERS} ${SOURCES})

target_link_libraries(sot_youbot dynamic-graph)
target_link_libraries(sot_youbot sot-core)
target_link_libraries(sot_youbot dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY(sot_youbot "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(sot_youbot "sot-core")
PKG_CONFIG_USE_DEPENDENCY(sot_youbot "dynamic-graph-python")


ADD_SUBDIRECTORY(src)
