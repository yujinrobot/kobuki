#!/bin/bash

# If we want a gui tool for this kind of thing, use something like zenity. e.g.
#     zenity --warning --text="This configuration tool must be run as root." --title="Turtlebot Config Tool"

##############################################################################
# Functions
##############################################################################

function check_is_root()
{
  if [[ $EUID -ne 0 ]]; then
    echo "This configuration tool must be run as root." 1>&2
    exit 1
  fi
}

function unplug_all()
{
  echo "Please unplug all USB cables."
}

function plug_kobuki()
{
  echo "Connect the Kobuki USB serial cable."
}

function plug_arm()
{
  echo "Connect TurtleBot Arm USB cable."
}

##############################################################################
# Main
##############################################################################

check_is_root

DEVICE=/dev/ttyUSB0

mkdir -p ~/.ros/udev_rules/

if [ -e /etc/udev/rules.d/57-kobuki.rules ]
then
  cp /etc/udev/rules.d/57-kobuki.rules ~/.ros/udev_rules/57-kobuki.rules.backup_$(date +%Y%m%d%P%I%M)
fi

# build udev rules
echo SUBSYSTEMS==\"usb\", ENV{ID_IFACE}=\"\$attr{bInterfaceNumber}\" > 57-usb-serial.rules.temp

idVendor=$(udevadm info -a -n /dev/ttyUSB0 | grep '{idVendor}' | head -n 1  | sed -e 's/==/ /g' -e 's/"//g' | awk '{print $2}')
idProduct=$(udevadm info -a -n /dev/ttyUSB0 | grep '{idProduct}' | head -n 1  | sed -e 's/==/ /g' -e 's/"//g' | awk '{print $2}')
serial=$(udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -n 1  | sed -e 's/==/ /g' -e 's/"//g' | awk '{print $2}')
bInterfaceNumber=$(udevadm info -a -n /dev/ttyUSB0 | grep '{bInterfaceNumber}' | head -n 1  | sed -e 's/==/ /g' -e 's/"//g' | awk '{print $2}')

echo $idVendor " " $idProduct " " $serial " " $bInterfaceNumber

echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"\
$idVendor\", ATTRS{idProduct}==\"\
$idProduct\", ATTRS{serial}==\"\
$serial\", ENV{ID_IFACE}==\"\
$bInterfaceNumber\", SYMLINK+=\"kobuki"\
"\"" 
#>> 57-kobuki.rules.temp

# display
#cat 57-usb-serial.rules.temp

# deploy 
#sudo cp 57-usb-serial.rules.temp /etc/udev/rules.d/57-usb-serial.rules

