/**
 * @file /cruizcore/src/nodelets/cruizcore_nodelet.cpp
 *
 * @brief Ros nodelet for the cruizcore driver.
 *
 * @date 20/08/2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <pluginlib/class_list_macros.h>
#include <ecl/streams/string_stream.hpp>
#include "kobuki_node/kobuki_node.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::StandardException;

/*****************************************************************************
** Implementation [KobukiNodelet]
*****************************************************************************/
/**
 * @brief Default constructor.
 *
 * Make sure you call the init() method to fully define this node.
 */
KobukiNodelet::KobukiNodelet() :
    //gyro_data(new device_comms::Gyro),
    slot_wheel_state (&KobukiNodelet::publishWheelState ,*this),
    slot_sensor_data(&KobukiNodelet::publishSensorData,*this),
	slot_default (&KobukiNodelet::publishDefaultData,*this),
	slot_ir      (&KobukiNodelet::publishIRData,*this),
	slot_dock_ir (&KobukiNodelet::publishDockIRData,*this),
	slot_inertia (&KobukiNodelet::publishInertiaData,*this),
	slot_cliff   (&KobukiNodelet::publishCliffData,*this),
	slot_current (&KobukiNodelet::publishCurrentData,*this),
	slot_magnet  (&KobukiNodelet::publishMagnetData,*this),
	slot_hw      (&KobukiNodelet::publishHWData,*this),
	slot_fw      (&KobukiNodelet::publishFWData,*this),
	slot_time    (&KobukiNodelet::publishTimeData,*this),
	slot_st_gyro (&KobukiNodelet::publishStGyroData,*this),
	slot_eeprom  (&KobukiNodelet::publishEEPROMData,*this),
	slot_gp_input(&KobukiNodelet::publishGpInputData,*this)/*,
		slot_reserved0(&KobukiNodelet::publishGpInputData,*this),*/
/*
    slot_invalid_packet(&CruizCoreNodelet::publishInvalidPacket,*this)*/ 
	//one slot for joint state of both wheels, two publisher for diff_drive_base
{
}
/**
 :* @brief Destructs, but only after the thread has cleanly terminated.
 *
 * Ensures we stay alive long enough for the thread to cleanly terminate.
 */
KobukiNodelet::~KobukiNodelet() {
    this->shutdown_requested = true;
    ROS_INFO_STREAM("Device : waiting for iclebo thread to finish [" << name << "].");
    iclebo.close();
    //iclebo.join(); 
}

/*****************************************************************************
** CruizCoreNodelet [DeviceNodelet Setup]
*****************************************************************************/
/**
 * @brief Initialises the cruizcore from the ros parameter server.
 *
 * To set up the cruizcore node you need to roslaunch it along with a parameter
 * configuration. That configuration is done from the parameter server here.
 * If configuration fails, it shuts down the ros process node.
 *
 * @param nh : private nodehandle to use for initialisation.
 */
bool KobukiNodelet::init(ros::NodeHandle& nh) {

	/*********************
	** Communications
	**********************/
	advertiseTopics(nh);
	subscribeTopics(nh);

	/*********************
	** Sigslots
	**********************/
	slot_wheel_state.connect(name+std::string("/joint_state"));
	slot_sensor_data.connect(name+std::string("/sensor_data"));
	slot_default.connect(name+std::string("/default")); 
	slot_ir.connect(name+std::string("/ir")); 
	slot_dock_ir.connect(name+std::string("/dock_ir")); 
	slot_inertia.connect(name+std::string("/inertia")); 
	slot_cliff.connect(name+std::string("/cliff")); 
	slot_current.connect(name+std::string("/current")); 
	slot_magnet.connect(name+std::string("/magnet")); 
	slot_hw.connect(name+std::string("/hw")); 
	slot_fw.connect(name+std::string("/fw")); 
	slot_time.connect(name+std::string("/time")); 
	slot_st_gyro.connect(name+std::string("/st_gyro")); 
	slot_eeprom.connect(name+std::string("/eeprom")); 
	slot_gp_input.connect(name+std::string("/gp_input")); 
	//slot_reserved0;


	/*********************
	** Parameters
	**********************/
	Parameters parameters;
	//gyro_data->header.frame_id = ""; // unused - add a parameter should we need it later.
	parameters.sigslots_namespace = name; // name is automatically picked up by device_nodelet parent.
	if ( !nh.getParam("device_id", parameters.device_id) ) {
		ROS_ERROR_STREAM("Device : no device id configuration on the parameter server [" << name << "].");
		return false;
	}
	if ( !nh.getParam("device_name", parameters.device_name) ) {
		ROS_ERROR_STREAM("Device : no device name given on the parameter server ('serial'||'ftdi')[" << name << "].");
		return false;
	}
	if ( !nh.getParam("device_port", parameters.device_port) ) {
		ROS_ERROR_STREAM("Device : no device name given on the parameter server ('serial'||'ftdi')[" << name << "].");
		return false;
	}
	if ( !nh.getParam("protocol_version", parameters.protocol_version) ) {
		ROS_ERROR_STREAM("Device : no protocol version given on the parameter server ('1.0'||'2.0')[" << name << "].");
		std::cout << "protocol_version: " << parameters.protocol_version << std::endl;
		return false;
	}

	/*********************
	** Validation
	**********************/
	if ( !parameters.validate() ) {
		ROS_ERROR_STREAM("Device : parameter configuration failed [" << name << "].");
		ROS_ERROR_STREAM("Device : " << parameters.error_msg << "[" << name << "]");
		return false;
	} else {
		ROS_INFO_STREAM("Device : parameters configured [" << parameters.device_name << "][" << parameters.device_id << "][" << name << "]");
		ROS_INFO_STREAM("Device : parameter.device_port [" << parameters.device_port << "]");
		ROS_INFO_STREAM("Device : parameter.protocol_version [" << parameters.protocol_version << "]");
	}

	/*********************
	** Driver Init
	**********************/
	try {
		iclebo.init(parameters);
	} catch ( const StandardException &e ) {
		switch( e.flag() ) {
			case(ecl::OpenError) : {
				ROS_ERROR_STREAM("Device : could not open connection [" << parameters.device_name << "][" << parameters.device_id << "][" << name << "].");
				break;
			}
			default : {
				ROS_ERROR_STREAM("Device : initialisation failed [" << name << "].");
				break;
			}
		}
		return false;
	}
	return true;
}

/**
 * @brief Sets up the cruizcore nodelet publications.
 *
 * These are put relative (on top) of the current handle's namespace,
 * e.g. cruizcore -> cruizcore/raw_data_sent
 *
 * Note, for a mobile base model to pick up gyro information, the topic name
 * *must* be 'mobile_base_gyro'.
 *
 * @param nh : the nodehandle derived from the parent nodelet.
 */
void KobukiNodelet::advertiseTopics(ros::NodeHandle& nh) {
	left_wheel_state_publisher  = nh.advertise<device_comms::JointState>("joint_state/left_wheel" , 100);
	right_wheel_state_publisher = nh.advertise<device_comms::JointState>("joint_state/right_wheel", 100);
	sensor_data_publisher 		= nh.advertise<iclebo_comms::iClebo>("sensor_data", 100);

	default_data_publisher		= nh.advertise<iclebo_comms::iClebo>("default_data", 100);	
	ir_data_publisher			= nh.advertise<iclebo_comms::iCleboIR>("ir_data", 100);
	dock_ir_data_publisher		= nh.advertise<iclebo_comms::iCleboDockIR>("dock_ir_data", 100);
	inertia_data_publisher		= nh.advertise<iclebo_comms::iCleboInertia>("inertia_data", 100);
	cliff_data_publisher		= nh.advertise<iclebo_comms::iCleboCliff>("cliff_data", 100);
	current_data_publisher		= nh.advertise<iclebo_comms::iCleboCurrent>("current_data", 100);
	magnet_data_publisher	 	= nh.advertise<iclebo_comms::iCleboMagnet>("merge_data", 100);
	hw_data_publisher			= nh.advertise<iclebo_comms::iCleboHW>("hw_data", 100);
	fw_data_publisher			= nh.advertise<iclebo_comms::iCleboFW>("fw_data", 100);
	time_data_publisher		  	= nh.advertise<iclebo_comms::iCleboTime>("time_data", 100);
	st_gyro_data_publisher		= nh.advertise<iclebo_comms::iCleboStGyro>("st_gyro_data", 100);
	eeprom_data_publisher		= nh.advertise<iclebo_comms::iCleboEEPROM>("eeprom_data", 100);
	gp_input_data_publisher		= nh.advertise<iclebo_comms::iCleboGpInput>("gp_input_data", 100);
	//reserved0_data_publish	= nh.advertise<iclebo_comms::iClebo>("default_data", 100);er 

	//invalid_packet_publisher = nh.advertise<std_msgs::String>("invalid_packets", 100);
	//gyro_data_publisher = nh.advertise<device_comms::Gyro>("gyro_data", 100);
}

/**
 * @brief Subscribes to topics relevant for cruizcore.
 *
 * These are found relative (on top) of the current handle's namespace,
 *
 * @param nh : the nodehandle derived from the parent nodelet.
 */
void KobukiNodelet::subscribeTopics(ros::NodeHandle& nh) {
	std::string	left_wheel_name  = "left_wheel";
	std::string	right_wheel_name = "right_wheel";

	left_wheel_command_subscriber  = nh.subscribe(std::string("joint_command/")+left_wheel_name,10,&KobukiNodelet::subscribeJointCommandLeft, this);
	right_wheel_command_subscriber = nh.subscribe(std::string("joint_command/")+right_wheel_name,10,&KobukiNodelet::subscribeJointCommandRight, this);
	velocity_command_subscriber	   = nh.subscribe(std::string("cmd_vel"),10,&KobukiNodelet::subscribeVelocityCommand, this);
	iclebo_command_subscriber	   = nh.subscribe(std::string("iclebo_command"),10,&KobukiNodelet::subscribeiCleboCommand, this);
}


/*****************************************************************************
** Implementation [CruizCoreNodelet][Threads]
*****************************************************************************/
/**
 * @brief The cruizcore node's io processing loop (run in the thread).
 *
 * This loop runs in a background thread - this is necessary as you may often
 * wish to have differen't cruizcore's running at different scan frequencies (e.g.
 * arm cruizcore at high frequency, peripheral head/scanner motors at low frequency).
 *
 * Note that intermittent sends will intersperse themselves within a scan.
 *
 * The type of commands sent to each motor (board) is identified by the feedback command held
 * in each motor.
 */
void KobukiNodelet::publishWheelState() {

	//waitForInitialisation();
	if ( ros::ok() && !shutdown_requested ) {
		if (left_wheel_state_publisher.getNumSubscribers() > 0 ) {
			iclebo.pubtime("  left_wheel:ent");
			device_comms::JointState joint_state;
			joint_state.name="left_wheel";
			joint_state.stamp = ros::Time::now();
			iclebo.getJointState(joint_state);
			left_wheel_state_publisher.publish(joint_state);
			iclebo.pubtime("  left_wheel:pub");
		}
		if (right_wheel_state_publisher.getNumSubscribers() > 0 ) {
			iclebo.pubtime("  right_wheel:ent");
			device_comms::JointState joint_state;
			joint_state.name="right_wheel";
			joint_state.stamp = ros::Time::now();
			iclebo.getJointState(joint_state);
			right_wheel_state_publisher.publish(joint_state);
			iclebo.pubtime("  right_wheel:pub");
		}
	}
	//ROS_INFO_STREAM("Device : thread terminating [" << name << "]");
}


/*****************************************************************************
** CruizCoreNodelet Publications
*****************************************************************************/
/**
 * @brief Publishes byte representations of raw data packets that are received.
 *
 * This is a callback for the signal sent by the cruizcore driver whenever
 * a raw packet is received.
 *
 * @param bytes : the raw byte array that is being recieved.
 */
void KobukiNodelet::publishSensorData() {

	if ( ros::ok() ) {
		if ( sensor_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iClebo data;
			iclebo.getData2(data);
			data.header.stamp = ros::Time::now();
			sensor_data_publisher.publish(data);
			//std::cout << "publishSensorData()" << std::endl;
		}
	}
}

void KobukiNodelet::subscribeJointCommandLeft(const device_comms::JointCommand cmd)
{
	//cmd.value;
	return;
}

void KobukiNodelet::subscribeJointCommandRight(const device_comms::JointCommand cmd)
{
	//cmd.value;
	return;
}

void KobukiNodelet::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr &msg)
{
    if( iclebo.isEnabled() ) {
		// For now assuming this is in the robot frame, but probably this
		// should be global frame and require a transform
		//double vx = msg->linear.x;	// in (m/s)
		//double wz = msg->angular.z;	// in (rad/s)
		ROS_DEBUG_STREAM("subscribeVelocityCommand: [" << msg->linear.x << "],[" << msg->angular.z << "]");
		iclebo.setCommand(msg->linear.x, msg->angular.z);
    } else {
        ROS_WARN("Robot is not enabled.");
    }
	return;
}

void KobukiNodelet::subscribeiCleboCommand(const iclebo_comms::iCleboCommandConstPtr &msg)
{
    //if( iclebo.isEnabled() ) {
		iclebo.sendCommand( msg );
	//} else {
	//	ROS_WARN("Robot is not enabled.");
	//}
	return;
}

/**
 * @brief Publishes byte representations of raw data packets that are sent.
 *
 * This is a callback for the signal sent by the cruizcore driver whenever
 * a raw packet is sent.
 *
 * @param bytes : the raw byte array that is being sent.
 */
#if 0
void CruizCoreNodelet::publishRawDataSent( const Packet::BufferStencil &bytes ) {

	if ( ros::ok() ) {
		if ( raw_data_sent_publisher.getNumSubscribers() > 0 ) {
			ecl::Format<Packet::Buffer> format;
			ecl::StringStream sstream;
			sstream << format(bytes);
			std_msgs::String msg;
			msg.data = sstream.str();
			raw_data_sent_publisher.publish(msg);
		}
	}
}
#endif
/**
 * @brief Publishes byte representations of invalid packets that are received.
 *
 * This is a callback for the signal sent by the cruizcore driver whenever
 * an invalid packet is received.
 *
 * @param bytes : the raw byte array that is the invalid packet being recieved.
 */
#if 0
void CruizCoreNodelet::publishInvalidPacket( const Packet::BufferStencil &bytes ) {

	if ( ros::ok() ) {
		if ( invalid_packet_publisher.getNumSubscribers() > 0 ) {
			ecl::Format<Packet::Buffer> format;
			ecl::StringStream sstream;
			sstream << format(bytes);
			std_msgs::String msg;
			msg.data = sstream.str();
			invalid_packet_publisher.publish(msg);
		}
	}
}
#endif

void KobukiNodelet::publishDefaultData()
{
	if ( ros::ok() ) {
		if ( default_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iClebo data;
			iclebo.getDefaultData(data);
			data.header.stamp = ros::Time::now();
			default_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishIRData()
{
	if ( ros::ok() ) {
		if ( ir_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboIR data;
			iclebo.getIRData(data);
			data.header.stamp = ros::Time::now();
			ir_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishDockIRData()
{
	if ( ros::ok() ) {
		if ( default_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iClebo data;
			iclebo.getDefaultData(data);
			data.header.stamp = ros::Time::now();
			default_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishInertiaData()
{
	if ( ros::ok() ) {
		if ( inertia_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboInertia data;
			iclebo.getInertiaData(data);
			data.header.stamp = ros::Time::now();
			inertia_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishCliffData()
{
	if ( ros::ok() ) {
		if ( cliff_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboCliff data;
			iclebo.getCliffData(data);
			data.header.stamp = ros::Time::now();
			cliff_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishCurrentData()
{
	if ( ros::ok() ) {
		if ( current_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboCurrent data;
			iclebo.getCurrentData(data);
			data.header.stamp = ros::Time::now();
			current_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishMagnetData()
{
	if ( ros::ok() ) {
		if ( magnet_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboMagnet data;
			iclebo.getMagnetData(data);
			data.header.stamp = ros::Time::now();
			magnet_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishHWData()
{
	if ( ros::ok() ) {
		if ( hw_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboHW data;
			iclebo.getHWData(data);
			data.header.stamp = ros::Time::now();
			hw_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishFWData()
{
	if ( ros::ok() ) {
		if ( fw_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboFW data;
			iclebo.getFWData(data);
			data.header.stamp = ros::Time::now();
			fw_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishTimeData()
{
	if ( ros::ok() ) {
		if ( time_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboTime data;
			iclebo.getTimeData(data);
			data.header.stamp = ros::Time::now();
			time_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishStGyroData()
{
	if ( ros::ok() ) {
		if ( st_gyro_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboStGyro data;
			iclebo.getStGyroData(data);
			data.header.stamp = ros::Time::now();
			st_gyro_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishEEPROMData()
{
	if ( ros::ok() ) {
		if ( eeprom_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboEEPROM data;
			iclebo.getEEPROMData(data);
			data.header.stamp = ros::Time::now();
			eeprom_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}

void KobukiNodelet::publishGpInputData()
{
	if ( ros::ok() ) {
		if ( gp_input_data_publisher.getNumSubscribers() > 0 ) {
			iclebo_comms::iCleboGpInput data;
			iclebo.getGpInputData(data);
			data.header.stamp = ros::Time::now();
			gp_input_data_publisher.publish(data);
			//std::cout << __func__ << std::endl;
		}
	}
}


/*		slot_reserved0, Rei*/
} // namespace k

/*****************************************************************************
** Nodelet Plugin Registration
*****************************************************************************/

PLUGINLIB_DECLARE_CLASS(kobuki_node, KobukiNodelet, kobuki::KobukiNodelet, nodelet::Nodelet);

