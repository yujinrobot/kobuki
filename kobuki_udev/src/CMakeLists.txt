##############################################################################
# Binaries
##############################################################################

rosbuild_add_executable( ftdi_scan ftdi_scan.cpp )
target_link_libraries(ftdi_scan ftdi)

rosbuild_add_executable( ftdi_kobuki ftdi_kobuki.cpp )
target_link_libraries(ftdi_kobuki ftdi)

rosbuild_add_executable(ftdi_read_eeprom ftdi_read_eeprom.cpp )
target_link_libraries(ftdi_read_eeprom ftdi)

rosbuild_add_executable(ftdi_write_eeprom ftdi_write_eeprom.cpp )
target_link_libraries(ftdi_write_eeprom ftdi)
