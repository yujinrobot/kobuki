cmake_minimum_required(VERSION 2.8.3)
project(kobuki_testsuite)
find_package(catkin REQUIRED COMPONENTS kobuki_driver kobuki_node std_msgs sensor_msgs geometry_msgs message_generation)

catkin_python_setup()

add_message_files(DIRECTORY msg
                  FILES ScanAngle.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

install(PROGRAMS scripts/inf_rotation.py
                 scripts/test_analog_input.py
                 scripts/test_battery_voltage.py
                 scripts/test_events.py
                 scripts/test_gyro.py
                 scripts/test_led_array.py
                 scripts/test_rotation.py
                 scripts/test_sounds.py
                 scripts/scan_angle.py
                 scripts/test_battery
                 scripts/test_digital_output.py
                 scripts/test_external_power.py
                 scripts/test_input.py
                 scripts/test_output.py
                 scripts/test_safewandering.py
                 scripts/test_translation.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
