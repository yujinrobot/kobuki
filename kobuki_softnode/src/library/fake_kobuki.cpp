#include <kobuki_softnode/fake_kobuki.h>

namespace kobuki
{
  void FakeKobuki::init(ros::NodeHandle& nh)
  {
    this->wheel_speed_cmd[LEFT] = 0.0;
    this->wheel_speed_cmd[RIGHT] = 0.0;

    // using the same values as in kobuki_node
    double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                          0, 0.1,   0,   0,   0, 0,
                          0,   0, 1e6,   0,   0, 0,
                          0,   0,   0, 1e6,   0, 0,
                          0,   0,   0,   0, 1e6, 0,
                          0,   0,   0,   0,   0, 0.2};
    memcpy(&(this->odom.pose.covariance),pcov,sizeof(double)*36);
    memcpy(&(this->odom.twist.covariance),pcov,sizeof(double)*36);

    // wheel information from kobuki_gazebo
    this->wheel_separation = 0.23;
    this->wheel_diameter = 0.070;

    // joint states
    nh.param("wheel_left_joint_name",this->wheel_joint_name[LEFT], std::string("wheel_left_joint"));
    nh.param("wheel_right_joint_name",this->wheel_joint_name[RIGHT], std::string("wheel_right_joint"));
    nh.param("cmd_vel_timeout",this->cmd_vel_timeout, 0.6);
    this->cmd_vel_timeout = 1.0;

    this->motor_enabled = true;

    this->joint_states.header.frame_id = "Joint States";
    this->joint_states.name.push_back(wheel_joint_name[LEFT]);
    this->joint_states.name.push_back(wheel_joint_name[RIGHT]);
    this->joint_states.position.resize(2,0.0);
    this->joint_states.velocity.resize(2,0.0);
    this->joint_states.effort.resize(2,0.0);

    // odometry
    nh.param("odom_frame",this->odom.header.frame_id,std::string("odom"));
    nh.param("base_frame",this->odom.child_frame_id,std::string("base_footprint"));

    this->versioninfo.hardware = "dumb";
    this->versioninfo.firmware = "fake";
    this->versioninfo.software = "0.0.0";

    this->odom_pose[0] = 0;
    this->odom_pose[1] = 0;
    this->odom_pose[2] = 0;

  }
}
