cmake_minimum_required(VERSION 2.8.3)
project(kobuki_random_walker)
find_package(catkin REQUIRED COMPONENTS ecl_threads
                                        geometry_msgs
                                        kobuki_msgs
                                        nodelet
                                        pluginlib
                                        roscpp
                                        std_msgs
                                        yocs_controllers)

catkin_package(INCLUDE_DIRS include
               LIBRARIES kobuki_random_walker_nodelet
               CATKIN_DEPENDS ecl_threads
                              geometry_msgs
                              kobuki_msgs
                              nodelet
                              pluginlib
                              roscpp
                              std_msgs
                              yocs_controllers)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_library(kobuki_random_walker_nodelet src/nodelet.cpp)
add_dependencies(kobuki_random_walker_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(kobuki_random_walker_nodelet ${catkin_LIBRARIES})

install(TARGETS kobuki_random_walker_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
