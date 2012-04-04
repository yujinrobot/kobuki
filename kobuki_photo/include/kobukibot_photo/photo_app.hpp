#ifndef PHOTO_APP_HPP_
#define PHOTO_APP_HPP_

#include <ros/ros.h>

#include <image_transport/image_transport.h>	
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/CvBridge.h>

#include <sensor_msgs/Image.h>		
#include <std_msgs/Empty.h> 		
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>		

#include <remote_mutex/RemoteMutex.h>

#include <ecl/Eigen3/Geometry>
#include <ecl/geometry.hpp>

namespace photo_app {

enum State
{
	SNAP,
	IDLE,
	TRACK,
	STOP,
	DIRECT,
};

class PhotoApp
{
public:
	  PhotoApp();
	  ~PhotoApp();

	  void init();
	  void spin();

private:
	sensor_msgs::CvBridge img_bridge_;

	ros::Subscriber sub_take_photo;
	ros::Subscriber sub_set_path;
	
	ros::Subscriber sub_track_nearest;
	ros::Subscriber sub_direct_nearest;

	image_transport::Publisher pub_photo;
	image_transport::Subscriber sub_get_image;
	image_transport::Subscriber sub_get_depth_image;
	
	ros::Subscriber sub_odom;
	ros::Publisher pub_cmd_vel;
	ros::Publisher pub_power_enable;
	ros::Publisher pub_power_disable;
	ros::Publisher pub_tilt_command;


  
	void takePhoto( const std_msgs::EmptyConstPtr& msg );
	void setPath( const std_msgs::EmptyConstPtr& msg );
	void savePhoto( const sensor_msgs::ImageConstPtr& msg );

	void directNearestPoint( const std_msgs::EmptyConstPtr& msg );
	void trackNearestPoint( const std_msgs::EmptyConstPtr& msg );
	void findNearestPoint( const sensor_msgs::ImageConstPtr& msg );

	void odomCb( const nav_msgs::OdometryConstPtr& msg );

  	double angle, last_angle;
	double target_z_;
	
	State m_nState;
	int m_nFrameNum;

	int rows_, cols_;	
	int min_i_, min_j_;
	int find_j_;
	float min_;


	std_msgs::String power_cmd;
	geometry_msgs::Twist cmd_vel;
	
	RemoteMutex *mutex_iclebo;

};

} //namespace pano_app

#endif /* PANO_APP_HPP_ */
