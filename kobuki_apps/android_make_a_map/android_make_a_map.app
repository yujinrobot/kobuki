display: Make a Map
description: Make a map by driving a Turtlebot from an Android device.
platform: turtlebot
launch: kobuki_apps/android_make_a_map.launch
interface: kobuki_apps/android_teleop.interface
icon: turtlebot_teleop/map.jpg
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.makeamap.MakeAMap
   app: 
     gravityMode: 0
     base_control_topic: /kobuki/cmd_vel 
