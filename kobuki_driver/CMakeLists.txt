cmake_minimum_required(VERSION 2.8.3)
project(kobuki_driver)
find_package(catkin REQUIRED COMPONENTS ecl_mobile_robot ecl_converters ecl_devices ecl_geometry ecl_sigslots ecl_time ecl_command_line)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki
   CATKIN_DEPENDS ecl_mobile_robot ecl_converters ecl_devices ecl_geometry ecl_sigslots ecl_time ecl_command_line
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_subdirectory(src)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
