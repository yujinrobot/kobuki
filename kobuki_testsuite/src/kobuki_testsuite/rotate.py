import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy
import math
from geometry_msgs.msg import Twist

def rotate_main():
    rospy.init_node("test_rotation")
    pub = rospy.Publisher('/cmd_vel',Twist)
    freq = 5
    rate = rospy.Rate(freq)
    twist = Twist()
    yaw_rate = 1.2
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    max_rotate_count = freq*int(3.14/yaw_rate)
    rotate_count = max_rotate_count
    start = rospy.get_rostime()
    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        if rotate_count == max_rotate_count:
            if twist.angular.z > 0:
                mod = -1.0
            else:
                mod = 1.0
            update = mod*yaw_rate/10.0
            while math.fabs(twist.angular.z) <= yaw_rate:
                twist.angular.z = twist.angular.z + update
                pub.publish(twist)
                rospy.sleep(0.04)
            # Make sure it is exact so the inequality in the while loop doesn't mess up next time around
            twist.angular.z = mod*yaw_rate
            rotate_count = 0
        else:
            rotate_count += 1
        now = rospy.get_rostime()
        rospy.loginfo("Rotate: %ds", now.secs - start.secs)
        pub.publish(twist)
        rate.sleep()
