##############################################################################
# INCLUDES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# EXECUTABLES
##############################################################################

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})

rosbuild_add_executable(kobuki_node ${SOURCES})
target_link_libraries(kobuki_node ${PCL_LIBRARIES})
