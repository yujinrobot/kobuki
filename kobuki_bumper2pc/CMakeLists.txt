cmake_minimum_required(VERSION 2.8.3)
project(kobuki_bumper2pc)
find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib pcl_ros kobuki_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki_bumper2pc_nodelet
   CATKIN_DEPENDS roscpp nodelet pluginlib pcl_ros kobuki_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(kobuki_bumper2pc_nodelet src/kobuki_bumper2pc.cpp)
target_link_libraries(kobuki_bumper2pc_nodelet ${catkin_LIBRARIES})

install(TARGETS kobuki_bumper2pc_nodelet
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
