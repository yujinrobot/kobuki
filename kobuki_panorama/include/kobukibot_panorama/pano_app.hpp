#ifndef PANO_APP_HPP_
#define PANO_APP_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pano_ros/PanoCaptureAction.h>

#include <image_transport/image_transport.h>	
#include <sensor_msgs/Image.h>		
#include <std_msgs/Empty.h> 		
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>		
#include <geometry_msgs/Twist.h>	
//#include <> // and some for modal logic

#include <ecl/Eigen3/Geometry>
#include <ecl/threads/thread.hpp>
#include <ecl/geometry.hpp>

#include <remote_mutex/RemoteMutex.h>

namespace pano_app {

enum State
{
  PENDING,
  SNAP,
  TURN,
  STOP,
  DONE,
};

class PanoApp
{
public:
  PanoApp();

  void init();
  void spin();

private:
  actionlib::SimpleActionClient<pano_ros::PanoCaptureAction> ac;

  ros::Publisher pub_snap;
  ros::Publisher pub_stop;
  ros::Publisher pub_cmd_vel;
  ros::Publisher pub_power_enable;
  ros::Publisher pub_power_disable;

  ros::Subscriber sub_odom;
  ros::Subscriber sub_pano;

  image_transport::Publisher  pub_stitched;
  image_transport::Subscriber sub_stitched;

  std_msgs::Empty empty;
  std_msgs::String power_cmd;
  geometry_msgs::Twist cmd_vel;

  State state;
  bool is_active;
  double angle, last_angle;

  void snap();
  void turn();
  void stop();
  void done();
  bool hasReached();

  void takeCb( const std_msgs::EmptyConstPtr& msg );
  void imageCb( const sensor_msgs::ImageConstPtr& msg );

  void odomCb( const nav_msgs::OdometryConstPtr& msg );

  void activeCb();
  void feedbackCb( const pano_ros::PanoCaptureFeedbackConstPtr& feedback );
  void doneCb( const actionlib::SimpleClientGoalState& state, const pano_ros::PanoCaptureResultConstPtr& result );

  RemoteMutex *mutex_iclebo;

};

} //namespace pano_app

#endif /* PANO_APP_HPP_ */
