/**
 * @file /include/teleop_core/keyop_core.hpp
 *
 * @brief The controlling node for remote operations on robot_core.
 *
 * @date 25/05/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KEYOP_CORE_NODE_HPP_
#define KEYOP_CORE_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <termios.h> // for keyboard input
#include <ecl/threads.hpp>
#include <geometry_msgs/Twist.h>  // for velocity commands
#include <geometry_msgs/TwistStamped.h>  // for velocity commands
#include <std_msgs/String.h> // for enable/disable commands
#include <kobuki_comms/KeyboardInput.h> // keycodes from remote teleops.

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace keyop_core {

/*****************************************************************************
** Interface
*****************************************************************************/
/**
 * @brief Keyboard remote control for our robot core (mobile base).
 *
 */
class KeyOpCore {
public:
	/*********************
	** C&D
	**********************/
	KeyOpCore();
        ~KeyOpCore();
	void init();

	/*********************
	** Runtime
	**********************/
	void spin();

private:
	ros::Subscriber keyinput_subscriber;
	ros::Publisher velocity_publisher;
	ros::Publisher stamped_velocity_publisher;
	ros::Publisher enable_publisher, disable_publisher;
	ros::ServiceClient reset_odometry_client;
	bool accept_incoming;
	bool power_status;
	geometry_msgs::TwistPtr cmd;
	geometry_msgs::TwistStampedPtr cmd_stamped;
	std_msgs::StringPtr power_cmd;
	double linear_vel_step, linear_vel_max;
	double angular_vel_step, angular_vel_max;
	std::string name;
        std::string mode;

	/*********************
	** Commands
	**********************/
	void enable();
	void disable();
	void incrementLinearVelocity();
	void decrementLinearVelocity();
	void incrementAngularVelocity();
	void decrementAngularVelocity();
	void resetVelocity();

	/*********************
	** Keylogging
	**********************/

	void keyboardInputLoop();
	void processKeyboardInput(char c);
	void remoteKeyInputReceived(const kobuki_comms::KeyboardInput& key);
	void restoreTerminal();
	bool quit_requested;
        int key_file_descriptor;
        struct termios original_terminal_state;
        ecl::Thread thread;
};

} // namespace keyop_core

#endif /* KEYOP_CORE_NODE_HPP_ */
