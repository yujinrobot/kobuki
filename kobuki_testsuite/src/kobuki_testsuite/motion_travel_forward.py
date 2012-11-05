import threading
import roslib; roslib.load_manifest('kobuki_testsuite')
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
    
'''
  Travels forward a set distance. 
'''
class TravelForward(object):
    '''
      Initialise everything

      API:
        start(xxx,xxx,xxx) - start travelling. 
        stop()  - stop.
      
      @param topic names
      @type strings
    '''
    def __init__(self, cmd_vel_topic, odom_topic):
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist)
        self._speed = 0.7
        self._distance = 1.0
        self._stop = False
        self._current_pose = None
        self._starting_pose = None
        self._run_thread = None
    
    def shutdown(self):
        self.stop()
        self.cmd_vel_publisher.unregister()
        self.odom_subscriber.unregister()

    def start(self, speed, distance, run_finished_callback):
        '''
          Set the speed and the travel distance, then launch the thread.
          
          Ignore the command if the thread is running.
        '''
        self._speed = speed
        self._distance = distance
        if self.is_alive():
            rospy.logerr("Kobuki TestSuite: already executing a motion, ignoring the request")
            return False
        else:
            self._starting_pose = self._current_pose
            self._run_thread = WorkerThread(self._run, run_finished_callback)
            self._run_thread.start()
            return True

    def stop(self):
        self._stop = True
        if self._run_thread:
            self._run_thread.join()
            self._run_thread = None
        cmd = Twist()
        cmd.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd)
        
    def _run(self):
        self._stop = False
        rate = rospy.Rate(10)
        self._current_speed = 0.0
        current_distance_sq = 0.0
        while not self._stop and not rospy.is_shutdown():
            if current_distance_sq < distance_sq:
                self.stop()
            else:
                current_distance_sq = (self._current_pose.position.x - self._starting_pose.position.x)*(self._current_pose.position.x - self._starting_pose.position.x) + \
                                   (self._current_pose.position.y - self._starting_pose.position.y)*(self._current_pose.position.y - self._starting_pose.position.y)
                #current_distance += 0.01
                print("Distance %s"%math.sqrt(current_distance_sq))
                if self._current_speed < speed:
                    self._current_speed += 0.01
                cmd = Twist()
                cmd.linear.x = self._current_speed
                self.cmd_vel_publisher.publish(cmd)
            rate.sleep()
        
    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def odometry_callback(self, data):
        self._current_pose = data.pose.pose
