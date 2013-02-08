cmake_minimum_required(VERSION 2.8.3)
project(kobuki_description)
find_package(catkin REQUIRED COMPONENTS urdf)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(REQUIRED xacro)

catkin_package(
   CATKIN_DEPENDS urdf 
   DEPENDS xacro
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
