cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/base.cmake)
include(cmake/python.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
#rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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
    src/youbot_controller_plugin.cpp
    src/youbot_sot_controller.cpp
    src/youbot_device.cpp)

set(HEADERS
    include/sot_youbot/youbot_controller_plugin.h
    include/sot_youbot/youbot_sot_controller.h
    include/sot_youbot/youbot_device.h)

rosbuild_add_library(sot_youbot SHARED ${HEADERS} ${SOURCES})

target_link_libraries(sot_youbot dynamic-graph)
target_link_libraries(sot_youbot sot-core)
target_link_libraries(sot_youbot dynamic-graph-python)
target_link_libraries(sot_youbot ros_bridge)
target_link_libraries(sot_youbot ros_interpreter)


rosbuild_add_library(sot_youbot ${HEADERS} ${SOURCES})

PKG_CONFIG_USE_DEPENDENCY(sot_youbot "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(sot_youbot "sot-core")
PKG_CONFIG_USE_DEPENDENCY(sot_youbot "dynamic-graph-python")


ADD_SUBDIRECTORY(src)
