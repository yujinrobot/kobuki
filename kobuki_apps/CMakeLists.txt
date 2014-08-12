cmake_minimum_required(VERSION 2.8.3)

project(kobuki_rapps)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY rapps DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
