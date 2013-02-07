cmake_minimum_required(VERSION 2.8.3)
project(kobuki_ftdi)
find_package(catkin REQUIRED COMPONENTS ecl_command_line)

# pkg-config packages
find_package(PkgConfig)
pkg_search_module(REQUIRED libusb-dev)
pkg_search_module(REQUIRED libftdi-dev)
pkg_search_module(REQUIRED ftdi-eeprom)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki_ros kobuki_nodelet
   CATKIN_DEPENDS ecl_command_line
   DEPENDS libusb-dev libftdi-dev ftdi-eeprom
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_subdirectory(src)

install(DIRECTORY bluetooth
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY eeproms
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/turtlebot_config
                 scripts/UNUSED
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
