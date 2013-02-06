##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# LIBRARIES
##############################################################################

add_library(kobuki ${SOURCES})
target_link_libraries(kobuki ${catkin_LIBRARIES})

install(TARGETS kobuki
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)