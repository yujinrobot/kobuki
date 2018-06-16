##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(kobuki_description)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake)

##############################################################################
# Installs
##############################################################################

install(DIRECTORY launch
        DESTINATION share
       )

install(DIRECTORY launch
        DESTINATION share
       )

install(DIRECTORY meshes
        DESTINATION share
       )

install(DIRECTORY rviz
        DESTINATION share
       )

