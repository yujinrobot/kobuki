#!/bin/bash
#AUTHOR: Younghun Ju <yhju@yujinrobot.com>, <yhju83@gmail.com>

test_angle=360.0
angle=-150.0

for test_angle in `seq 360 360.0 $((360*4))`; do
  for angle in `seq -150 5.0 150`; do
    if [ $angle == 0.0 ]; then continue; fi
    rosrun kobuki_testsuite gyro_perf.py\
      cmd_vel:=/mobile_base/commands/velocity\
      imu_data:=/mobile_base/sensors/imu_data\
      angle_abs:=/scan_angle\
      sound:=/mobile_base/commands/sound\
      button:=/mobile_base/events/button\
      _command_vx:=0.0\
      _command_wz:=$angle\
      _max_sample:=100\
      _test_angle:=$test_angle
    echo '--------------------------------------------------------------------------------'
    sleep 10
  done
  echo '================================================================================'
done
