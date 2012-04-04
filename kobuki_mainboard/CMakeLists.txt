##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

# Verbosity is useful - helps eclipse find paths also!
set(CMAKE_VERBOSE_MAKEFILE true)

##############################################################################
# YCS-ROS
##############################################################################

include($ENV{YCS_ROOT}/ycs_tools/ycs_build/ycsbuild.cmake)

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

ycsbuild_init()

##############################################################################
# Messages/Services
##############################################################################

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

##############################################################################
# Actual project configuration
##############################################################################

# Including a source directory
add_subdirectory(src)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

