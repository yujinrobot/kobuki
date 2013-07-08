cmake_minimum_required(VERSION 2.8.3)
project(kobuki_softnode)

find_package(catkin REQUIRED COMPONENTS nav_msgs sensor_msgs geometry_msgs kobuki_msgs roscpp nodelet tf)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki_softnode softkobuki_nodelet
   CATKIN_DEPENDS nav_msgs sensor_msgs geometry_msgs kobuki_msgs roscpp nodelet tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(kobuki_softnode src/library/fake_kobuki_ros.cpp
                            src/library/fake_kobuki.cpp
)
target_link_libraries(kobuki_softnode ${catkin_LIBRARIES})

add_library(softkobuki_nodelet src/nodelet/kobuki_nodelet.cpp)
target_link_libraries(softkobuki_nodelet kobuki_softnode)


install(TARGETS kobuki_softnode
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS softkobuki_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
