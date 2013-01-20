###############################################################################
# Tools
###############################################################################

rosbuild_add_executable(version_info version_info.cpp)
target_link_libraries(version_info kobuki)
