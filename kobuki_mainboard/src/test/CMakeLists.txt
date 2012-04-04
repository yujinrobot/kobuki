##############################################################################
# Files
##############################################################################

#file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

rosbuild_add_executable(test_mainboard test_mainboard.cpp)

#rosbuild_add_executable(bridge node.cpp)
#target_link_libraries(bridge cse_rviz_bridge)
