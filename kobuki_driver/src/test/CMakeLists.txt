###############################################################################
# Tests
###############################################################################

rosbuild_add_executable(velocity_commands velocity_commands.cpp)
target_link_libraries(velocity_commands kobuki)

rosbuild_add_executable(demo_initialisation initialisation.cpp)
target_link_libraries(demo_initialisation kobuki)

rosbuild_add_executable(demo_sigslots sigslots.cpp)
target_link_libraries(demo_sigslots kobuki)

rosbuild_add_executable(demo_simple_loop simple_loop.cpp)
target_link_libraries(demo_simple_loop kobuki)

rosbuild_add_executable(version_info version_info.cpp)
target_link_libraries(version_info kobuki)
