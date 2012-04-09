##############################################################################
# Binaries
##############################################################################

rosbuild_add_executable( ftdi_scan_id ftdi_scan_id.cpp )
target_link_libraries(ftdi_scan_id ftdi)

rosbuild_add_executable( ftdi_write_id ftdi_write_id.cpp )
target_link_libraries(ftdi_write_id ftdi)

#rosbuild_add_executable(ftdi_read ftdi_read.cpp )
#target_link_libraries(ftdi_read ros_ftdi)
