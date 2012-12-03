#!/bin/bash

rostopic pub /mobile_base/commands/cancel_dock std_msgs/String 'all' -1
