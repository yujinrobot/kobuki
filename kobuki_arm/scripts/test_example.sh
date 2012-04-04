#!/bin/bash

rostopic pub /gripper_controller/command std_msgs/Float64 -1 -- 0 &
rostopic pub /wrist_controller/command std_msgs/Float64 -1 -- 0 &
rostopic pub /elbow_controller/command std_msgs/Float64 -1 -- 0 &
rostopic pub /shoulder_controller/command std_msgs/Float64 -1 -- 0 &
rostopic pub /base_controller/command std_msgs/Float64 -1 -- 0 &
