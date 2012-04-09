#!/bin/bash

# backup
mkdir ~/.ycs/tools/udev_rules/ -p 

if [ -e /etc/udev/rules.d/57-usb-serial.rules ]
then
  cp /etc/udev/rules.d/57-usb-serial.rules ~/.ycs/tools/udev_rules/57-usb-serial.rules.backup_$(date +%Y%m%d%P%I%M)
fi


# build udev rules
echo SUBSYSTEMS==\"usb\", ENV{ID_IFACE}=\"\$attr{bInterfaceNumber}\" > 57-usb-serial.rules.temp

devices_array="/dev/ttyUSB0,CruizCore,Gyro,CC
/dev/ttyUSB1,LedBoard,HRI
/dev/ttyUSB2,Reserved0
/dev/ttyUSB3,RemoteController,RFRemocon
/dev/ttyUSB4,SensorIntegrationBoard,SIB
/dev/ttyUSB5,StarGazer,SG
/dev/ttyUSB6,Reserved1
/dev/ttyUSB7,Reserved2
/dev/ttyUSB8,GooWheel,Goo0
/dev/ttyUSB9,GooTurret,Goo1
/dev/ttyUSB10,GooArm,Goo2
/dev/ttyUSB11,GooHead,Goo3
/dev/ttyACM0,HokuyoLaserScanner,Hokuyo,LaserScanner"

for devices in $devices_array; do
  #echo $devices
  device=( $( echo $devices | tr , \ ) )
  #echo ${#device[@]}

  if [ -e ${device[0]} ]; then
    if [ ${#device[@]} -gt "1" ]; then
      idVendor=$(udevadm info -a -n ${device[0]} | grep '{idVendor}' | head -n 1  | sed -e 's/==/ /g' -e 's/"//g' | awk '{print $2}')
      idProduct=$(udevadm info -a -n ${device[0]} | grep '{idProduct}' | head -n 1  | sed -e 's/==/ /g' -e 's/"//g' | awk '{print $2}')
      serial=$(udevadm info -a -n ${device[0]} | grep '{serial}' | head -n 1  | sed -e 's/==/ /g' -e 's/"//g' | awk '{print $2}')
      bInterfaceNumber=$(udevadm info -a -n ${device[0]} | grep '{bInterfaceNumber}' | head -n 1  | sed -e 's/==/ /g' -e 's/"//g' | awk '{print $2}')
      #echo $idVendor " " $idProduct " " $serial " " $bInterfaceNumber
      for index in $(seq 1 $(( ${#device[@]}-1 )) ); do
        echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"\
$idVendor\", ATTRS{idProduct}==\"\
$idProduct\", ATTRS{serial}==\"\
$serial\", ENV{ID_IFACE}==\"\
$bInterfaceNumber\", SYMLINK+=\"serial/by-device/"\
${device[$index]}"\"" >> 57-usb-serial.rules.temp
      done
    else
      echo "[WARN] there are no symlink candidates for "${device[0]}" ."
    fi
  else
    echo "[WARN] device is not exist[ "${device[0]}" ]."
  fi
done

# dislpay
cat 57-usb-serial.rules.temp

# deploy 
sudo cp 57-usb-serial.rules.temp /etc/udev/rules.d/57-usb-serial.rules

