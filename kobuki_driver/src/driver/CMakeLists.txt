##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# CONFIGURATIONS
##############################################################################

set(VERSION_FILE ${PROJECT_SOURCE_DIR}/build/version_info.cpp)
message("making version ${kobuki_driver_VERSION}.")
configure_file(version_info.cpp.in ${VERSION_FILE} @ONLY)

##############################################################################
# LIBRARIES
##############################################################################

add_library(kobuki ${SOURCES} ${VERSION_FILE})
target_link_libraries(kobuki ${catkin_LIBRARIES})

install(TARGETS kobuki
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

