###############################################################################
# Tools
###############################################################################

add_executable(version_info version_info.cpp)
target_link_libraries(version_info kobuki)

install(TARGETS version_info
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
