##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.6.0)

# Verbosity is useful - helps eclipse find paths also!
set(CMAKE_VERBOSE_MAKEFILE true)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

##############################################################################
# YCS-ROS
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

##############################################################################
# Build
##############################################################################

# You should keep this commented for svn as ros (ycs) build options are 
# set globally. It is however, useful when debugging/testing for a single 
# package. The default option if no global ros configuration is set, is 
# RelWithDebInfo.
# 
# Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE RelWithDebInfo) 

##############################################################################
# Init - the order of the preceding commands is important!
##############################################################################

rosbuild_init()

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(src)
