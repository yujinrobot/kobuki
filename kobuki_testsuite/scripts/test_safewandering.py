#!/usr/bin/env python
import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy

from kobuki_testsuite import SafeWandering

if __name__ == '__main__':

    cmdvel_topic = '/cmd_vel'
    odom_topic =  '/odom'
    bump_topic = '/mobile_base/events/bumper'
    cliff_topic = '/mobile_base/events/cliff'
    rospy.init_node('safe_wandering')
    
    wanderer = SafeWandering(cmdvel_topic,odom_topic, bump_topic, cliff_topic)

    rospy.loginfo("Starting to wander")
    wanderer.start()
    rospy.sleep(5)
    rospy.loginfo("Slowing down")
    wanderer.set_vels(0.1,-0.05,0.5)
    rospy.sleep(5)
    rospy.loginfo("Stopping wandering")
    wanderer.stop()

