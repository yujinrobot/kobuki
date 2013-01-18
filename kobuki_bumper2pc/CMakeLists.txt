##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_VERBOSE_MAKEFILE TRUE) # useful for eclipse indexing

##############################################################################
# Ros
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

##############################################################################
# Nodelet
##############################################################################

rosbuild_add_library(kobuki_bumper2pc_nodelet src/kobuki_bumper2pc.cpp)
set_target_properties(kobuki_bumper2pc_nodelet PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
