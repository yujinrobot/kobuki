##############################################################################
# Binaries
##############################################################################

rosbuild_add_executable(ftdi_scan ftdi_scan.cpp)
target_link_libraries(ftdi_scan ftdi)

rosbuild_add_executable(ftdi_kobuki ftdi_kobuki.cpp)
target_link_libraries(ftdi_kobuki ftdi)

rosbuild_add_executable(ftdi_read_eeprom ftdi_read_eeprom.cpp)
target_link_libraries(ftdi_read_eeprom ftdi)

rosbuild_add_executable(ftdi_write_eeprom ftdi_write_eeprom.cpp)
target_link_libraries(ftdi_write_eeprom ftdi)

rosbuild_add_executable(find_devices find_devices.cpp)
target_link_libraries(find_devices usb)

rosbuild_add_executable(get_serial_number get_serial_number.cpp)
target_link_libraries(get_serial_number usb)

rosbuild_add_executable(flasher flasher.cpp)
target_link_libraries(flasher usb ftdi)

rosbuild_add_executable(reset_device reset_device.cpp)
target_link_libraries(reset_device usb)

rosbuild_add_executable(overwrite_serial_number overwrite_serial_number.cpp)
target_link_libraries(overwrite_serial_number usb ftdi)
