cmake_minimum_required(VERSION 2.8.3)
project(kobuki_keyop)
find_package(catkin REQUIRED COMPONENTS geometry_msgs std_srvs std_msgs roscpp ecl_exceptions ecl_threads ecl_time kobuki_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki
   CATKIN_DEPENDS geometry_msgs std_srvs std_msgs roscpp ecl_exceptions ecl_threads ecl_time kobuki_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_subdirectory(src)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
