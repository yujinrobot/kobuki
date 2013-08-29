cmake_minimum_required(VERSION 2.8.3)
project(kobuki_arm)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/arm_controller.py
                 scripts/joint_command_publisher
                 scripts/test_example.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
