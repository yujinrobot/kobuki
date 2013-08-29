##############################################################################
# NODELET
##############################################################################

add_library(kobuki_nodelet kobuki_nodelet.cpp)
add_dependencies(kobuki_nodelet kobuki_ros)
target_link_libraries(kobuki_nodelet kobuki_ros)

install(TARGETS kobuki_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)