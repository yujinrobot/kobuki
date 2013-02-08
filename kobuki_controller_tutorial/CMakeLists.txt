cmake_minimum_required(VERSION 2.8.3)
project(kobuki_controller_tutorial)
find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib std_msgs kobuki_msgs yocs_controllers)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES bump_blink_controller_nodelet
   CATKIN_DEPENDS roscpp nodelet pluginlib std_msgs kobuki_msgs yocs_controllers
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(bump_blink_controller_nodelet src/nodelet.cpp)
target_link_libraries(bump_blink_controller_nodelet ${catkin_LIBRARIES})

install(TARGETS bump_blink_controller_nodelet
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
