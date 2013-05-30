cmake_minimum_required(VERSION 2.8.3)
project(kobuki_shooter)

find_package(catkin REQUIRED COMPONENTS ecl_threads
                                        geometry_msgs
                                        kobuki_msgs
                                        nodelet
                                        pluginlib
                                        roscpp
                                        yocs_controllers)

catkin_package(INCLUDE_DIRS include
               LIBRARIES kobuki_shooter_controller_nodelet
               CATKIN_DEPENDS ecl_threads
                              geometry_msgs
                              kobuki_msgs
                              nodelet
                              pluginlib
                              roscpp
                              yocs_controllers)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_library(kobuki_shooter_controller_nodelet src/nodelet.cpp)
target_link_libraries(kobuki_shooter_controller_nodelet ${catkin_LIBRARIES})

install(TARGETS kobuki_shooter_controller_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})