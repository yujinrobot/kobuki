##############################################################################
# EXECUTABLES
##############################################################################

rosbuild_add_executable(kobuki_node kobuki_node.cpp)
target_link_libraries(kobuki_node kobuki_ros)
