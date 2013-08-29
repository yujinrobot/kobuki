###############################################################################
# Tools
###############################################################################

add_executable(version_info version_info.cpp)
target_link_libraries(version_info kobuki)

add_executable(simple_keyop simple_keyop.cpp)
target_link_libraries(simple_keyop kobuki)

install(TARGETS version_info simple_keyop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
