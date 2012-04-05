/**
 * @file /cruizcore/include/cruizcore/cruizcore_nodelet.hpp
 *
 * @brief Interface for the ros nodelet wrapper around cruizcore.
 *
 * @date 20/08/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_NODELET_HPP_
#define KOBUKI_NODELET_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <device_nodelet/device_nodelet.hpp>
#include <geometry_msgs/Twist.h>
#include <device_comms/JointCommand.h>
#include <device_comms/JointState.h>
#include <kobuki_comms/SensorData.h>
#include <ecl/sigslots.hpp>
#include <ecl/threads/thread.hpp>
#include <standard_comms/StringString.h>
#include "kobuki.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interface
*****************************************************************************/
/**
 * @brief Pseudo-node for the cruizcore.
 *
 * Configures ros communication types but does not implement the spin loop.
 * This is typically used as part of a library by another process, so the
 * spin loop is adapted elsewhere.
 *
 * This does, however implement all of the cruizcore functionality via the
 * composited cruizcore object. In addition,
 * control loop functionality can be achieved via calling the processInputs()
 * and processOutputs() methods.
 */
class KobukiNodelet : public device_interface::DeviceNodelet {
public:
    /*********************
    ** C&D
    **********************/
    KobukiNodelet();
    ~KobukiNodelet();

private:
	/******************************************
	** Device Nodelet Setup
	*******************************************/
	bool init(ros::NodeHandle& nh);
	void advertiseTopics(ros::NodeHandle& nh);
	void subscribeTopics(ros::NodeHandle& nh);

    	// Don't need to override the device nodelet enable/disable topic callbacks

	Kobuki kobuki;
	
	//KobukiMainboardDriver kobuki_receiver();
	//KobukiMainboardData data;
	//example
	//kobuki.getData(data);
        //left_wheels=data.encoder[0];

	/*********************
	** Ros Comms
	**********************/
	ros::Publisher left_wheel_state_publisher;
	ros::Publisher right_wheel_state_publisher;
	ros::Publisher sensor_data_publisher;
	ros::Subscriber left_wheel_command_subscriber, right_wheel_command_subscriber;
	ros::Subscriber velocity_command_subscriber;
	ros::Subscriber kobuki_command_subscriber;

	ecl::Slot<> slot_wheel_state, slot_sensor_data;
	ecl::Signal< const device_comms::JointCommand > sig_joint_command;
	// [ 2 ]
	ros::Publisher
		default_data_publisher, 
		ir_data_publisher,       
		dock_ir_data_publisher,  
		inertia_data_publisher,  
		cliff_data_publisher,    
		current_data_publisher,  
		magnet_data_publisher,   
		hw_data_publisher,       
		fw_data_publisher,       
		time_data_publisher,     
		st_gyro_data_publisher,  
		eeprom_data_publisher,   
		gp_input_data_publisher;
//		reserved0_data_publisher ;

	ecl::Slot<> 
		slot_default ,
		slot_ir      ,
		slot_dock_ir ,
		slot_inertia ,
		slot_cliff   ,
		slot_current ,
		slot_magnet  ,
		slot_hw      ,
		slot_fw      ,
		slot_time    ,
		slot_st_gyro ,
		slot_eeprom  ,
		slot_gp_input;
//		slot_reserved0;

	/*********************
	** SigSlots
	**********************/
	void publishWheelState();
	void publishSensorData();
	void publishDefaultData();
	void publishIRData();
	void publishDockIRData();
	void publishInertiaData();
	void publishCliffData();
	void publishCurrentData();
	void publishMagnetData();
	void publishHWData();
	void publishFWData();
	void publishTimeData();
	void publishStGyroData();
	void publishEEPROMData();
	void publishGpInputData();
	void subscribeJointCommandLeft(const device_comms::JointCommand);
	void subscribeJointCommandRight(const device_comms::JointCommand);
	void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr&);
	void subscribeKobukiCommand(const kobuki_comms::CommandConstPtr&);

	void enable() 
	{ 
		kobuki.run();
		kobuki.reset();
		ROS_INFO_STREAM( "kobuki enabled." ); 
	};

	void disable(){ 
		kobuki.stop();
		ROS_INFO_STREAM( "kobuki disable." ); 
	};
};

} // namespace cruizcore


#endif /* YCS_DEVICES_CRUIZECORE_NODELET_HPP_ */
