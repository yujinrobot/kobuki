cmake_minimum_required(VERSION 2.8.3)
project(kobuki_auto_docking)
find_package(catkin REQUIRED COMPONENTS roscpp rospy nodelet pluginlib message_filters actionlib kdl_conversions
                                        ecl_threads ecl_geometry ecl_linear_algebra kobuki_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki_auto_docking_ros kobuki_auto_docking_nodelet
   CATKIN_DEPENDS roscpp rospy nodelet pluginlib message_filters actionlib kdl_conversions
                  ecl_threads ecl_geometry ecl_linear_algebra kobuki_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(kobuki_auto_docking_ros src/auto_docking_ros.cpp src/dock_drive.cpp)
add_library(kobuki_auto_docking_nodelet src/nodelet.cpp)

add_dependencies(kobuki_auto_docking_ros kobuki_msgs_gencpp)

target_link_libraries(kobuki_auto_docking_ros ${catkin_LIBRARIES})
target_link_libraries(kobuki_auto_docking_nodelet ${catkin_LIBRARIES} kobuki_auto_docking_ros)

install(TARGETS kobuki_auto_docking_ros kobuki_auto_docking_nodelet
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
install(PROGRAMS scripts/DockDriveActionClient.py
                 scripts/DockDriveControl.py
                 scripts/getDockIR.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
