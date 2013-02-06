##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# LIBRARY
##############################################################################

add_library(kobuki_ros ${SOURCES})
target_link_libraries(kobuki_ros ${catkin_LIBRARIES})

install(TARGETS kobuki_ros
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
