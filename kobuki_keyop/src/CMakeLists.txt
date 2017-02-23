##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(keyop ${SOURCES})
add_dependencies(keyop ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyop ${catkin_LIBRARIES})

install(TARGETS keyop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
