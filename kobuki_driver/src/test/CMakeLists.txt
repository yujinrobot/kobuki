###############################################################################
# Tests
###############################################################################

add_executable(kobuki_velocity_commands velocity_commands.cpp)
target_link_libraries(kobuki_velocity_commands kobuki)

add_executable(demo_kobuki_initialisation initialisation.cpp)
target_link_libraries(demo_kobuki_initialisation kobuki)

add_executable(demo_kobuki_sigslots sigslots.cpp)
target_link_libraries(demo_kobuki_sigslots kobuki)

add_executable(demo_kobuki_simple_loop simple_loop.cpp)
target_link_libraries(demo_kobuki_simple_loop kobuki)

install(TARGETS kobuki_velocity_commands demo_kobuki_initialisation demo_kobuki_sigslots demo_kobuki_simple_loop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)