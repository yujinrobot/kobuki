##############################################################################
# INCLUDES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# LIBRARY
##############################################################################

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})

rosbuild_add_library(kobuki_ros ${SOURCES})
target_link_libraries(kobuki_ros ${PCL_LIBRARIES})
