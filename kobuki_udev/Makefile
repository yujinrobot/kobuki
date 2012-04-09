include $(shell rospack find mk)/cmake.mk

help:
	@echo ""
	@echo "Valid targets are: "
	@echo "  udev : copy udev rules for /dev/kobuki"
	@echo "  utils : create ftdi utilities."
	@echo ""

utils: all

udev:
	@echo ""
	@echo "Kobuki ftdi is preprogrammed with serial id 'kobuki'"
	@echo "Copying 57-kobuki.rules -> to /etc/udev/rules.d"
	@echo ""
	@sudo cp 57-kobuki.rules /etc/udev/rules.d
	@sudo service udev restart
	@echo ""

