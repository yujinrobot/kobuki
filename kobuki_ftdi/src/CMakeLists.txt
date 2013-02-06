##############################################################################
# Binaries
##############################################################################

add_executable(ftdi_scan ftdi_scan.cpp)
target_link_libraries(ftdi_scan ftdi)

add_executable(ftdi_kobuki ftdi_kobuki.cpp)
target_link_libraries(ftdi_kobuki ftdi)

add_executable(ftdi_read_eeprom ftdi_read_eeprom.cpp)
target_link_libraries(ftdi_read_eeprom ftdi)

add_executable(ftdi_write_eeprom ftdi_write_eeprom.cpp)
target_link_libraries(ftdi_write_eeprom ftdi)

add_executable(find_devices find_devices.cpp)
target_link_libraries(find_devices usb)

add_executable(get_serial_number get_serial_number.cpp)
target_link_libraries(get_serial_number usb)

add_executable(flasher flasher.cpp)
target_link_libraries(flasher usb ftdi)

add_executable(reset_device reset_device.cpp)
target_link_libraries(reset_device usb)

add_executable(overwrite_serial_number overwrite_serial_number.cpp)
target_link_libraries(overwrite_serial_number usb ftdi)

add_executable(unflasher unflasher.cpp)
target_link_libraries(unflasher usb ftdi)

install(TARGETS ftdi_scan ftdi_kobuki ftdi_read_eeprom ftdi_write_eeprom find_devices
                get_serial_number flasher reset_device overwrite_serial_number unflasher
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
