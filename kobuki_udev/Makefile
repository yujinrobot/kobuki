include $(shell rospack find mk)/cmake.mk

help:
	@echo ""
	@echo "Valid targets are: "
	@echo "  udev  : copy udev rules for /dev/kobuki"
	@echo "  utils : create ftdi utilities."
	@echo ""
	@echo "There is no make target for actually flashing the device."
	@echo "Because this is dangerous, be sure to read doxygen/mainpage.dox"
	@echo "for detailed info on how to do this by hand."
	@echo ""

utils: all

udev:
	@echo ""
	@echo "Copying 57-kobuki.rules -> to /etc/udev/rules.d"
	@echo ""
	@sudo cp 57-kobuki.rules /etc/udev/rules.d
	@sudo service udev reload
	@sudo service udev restart
	@echo ""

