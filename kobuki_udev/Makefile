#include $(shell rospack find mk)/cmake.mk

all:
	@echo ""
	@echo "Kobuki ftdi is preprogrammed with serial id 'kobuki'"
	@echo "Copying 57-kobuki.rules -> to /etc/udev/rules.d"
	@echo ""
	@sudo cp 57-kobuki.rules /etc/udev/rules.d
	@sudo service udev restart
	@echo ""

