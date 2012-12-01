##############################################################################
# NODELET
##############################################################################

rosbuild_add_library(kobuki_nodelet kobuki_nodelet.cpp)
target_link_libraries(kobuki_nodelet kobuki_ros)
