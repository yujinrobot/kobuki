###############################################################################
# Tests
###############################################################################

rosbuild_add_executable(velocity_commands velocity_commands.cpp)
target_link_libraries(velocity_commands kobuki)

