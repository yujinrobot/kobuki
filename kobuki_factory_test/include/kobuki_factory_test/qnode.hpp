/**
 * @file /include/kobuki_factory_test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_FACTORY_TEST_QNODE_HPP_
#define KOBUKI_FACTORY_TEST_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>

#include <QThread>
#include <QStringListModel>

#include <sensor_msgs/Imu.h>
#include <kobuki_comms/SensorState.h>
#include <kobuki_comms/VersionInfo.h>
#include <kobuki_comms/DockInfraRed.h>
#include <kobuki_comms/ButtonEvent.h>
#include <kobuki_comms/BumperEvent.h>
#include <kobuki_comms/WheelDropEvent.h>
#include <kobuki_comms/CliffEvent.h>
#include <kobuki_comms/PowerSystemEvent.h>
#include <kobuki_comms/DigitalInputEvent.h>
#include <kobuki_comms/RobotStateEvent.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include "../include/kobuki_factory_test/robots.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_factory_test {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  void run();

  enum EvalStep {
    DC_JACK_PLUGGED,
    DC_JACK_UNPLUGGED,
    DOCKING_PLUGGED,
    DOCKING_UNPLUGGED,
    BUTTON_0_PRESSED,
    BUTTON_0_RELEASED,
    BUTTON_1_PRESSED,
    BUTTON_1_RELEASED,
    BUTTON_2_PRESSED,
    BUTTON_2_RELEASED,
    TEST_LEDS,
    TEST_SOUNDS,
    CLIFF_AND_WHEEL_DROP,  // 10
    CENTER_BUMPER_PRESSED, // 11
    CENTER_BUMPER_RELEASED,
    POINT_RIGHT_BUMPER,
    RIGHT_BUMPER_PRESSED,
    RIGHT_BUMPER_RELEASED,
    POINT_LEFT_BUMPER,
    LEFT_BUMPER_PRESSED,  // 17
    LEFT_BUMPER_RELEASED,
    PREPARE_MOTORS_TEST,   // 19
    TEST_MOTORS_FORWARD,   // 20
    TEST_MOTORS_BACKWARD,
    TEST_MOTORS_CLOCKWISE,
    TEST_MOTORS_COUNTERCW,
    EVAL_MOTORS_CURRENT,

    MEASURE_GYRO_ERROR,
    MEASURE_CHARGING, //?

    SOMETHING_MORE,

    EVALUATION_STEPS_COUNT
  };

  /*********************
  ** Logging
  **********************/
  enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
   };

  QStringListModel* loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &format, ...);

Q_SIGNALS:
  void showMessage(const QString& title, const QString& text);
  void loggingUpdated();
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;

  bool     timer_active;
  ros::Timer timer;
  EvalStep current_step;

  Robot *   under_test;
  RobotList  evaluated;

  /*********************
  ** Publishers
  **********************/
  ros::Publisher cmd_vel_pub;
  ros::Publisher led_1_pub;
  ros::Publisher led_2_pub;
  ros::Publisher sound_pub;
  ros::Publisher output_pub;
  ros::Publisher ext_pwr_pub;

  /*********************
  ** Subscribers
  **********************/
  ros::Subscriber v_info_sub; // version info
  ros::Subscriber s_core_sub; // sensors core
  ros::Subscriber beacon_sub; // dock ir sensor
  ros::Subscriber gyro_sub;   // gyroscope data
  ros::Subscriber button_sub; // buttons events
  ros::Subscriber bumper_sub; // bumpers events
  ros::Subscriber w_drop_sub; // wheels drop events
  ros::Subscriber cliff_sub;  // cliff sensors events
  ros::Subscriber power_sub;  // power system events
  ros::Subscriber input_sub;  // digital input events
  ros::Subscriber robot_sub;  // robot state events
  ros::Subscriber state_sub;  // diagnostics top level state
  ros::Subscriber diags_sub;  // diagnostics

  QStringListModel logging_model;


  /*********************
  ** Callbacks
  **********************/
  void versionInfoCB(const kobuki_comms::VersionInfo::ConstPtr& msg);
  void sensorsCoreCB(const kobuki_comms::SensorState::ConstPtr& msg);
  void dockBeaconCB(const kobuki_comms::DockInfraRed::ConstPtr& msg);
  void gyroscopeCB(const sensor_msgs::Imu::ConstPtr& msg);
  void buttonEventCB(const kobuki_comms::ButtonEvent::ConstPtr& msg);
  void bumperEventCB(const kobuki_comms::BumperEvent::ConstPtr& msg);
  void wDropEventCB(const kobuki_comms::WheelDropEvent::ConstPtr& msg);
  void cliffEventCB(const kobuki_comms::CliffEvent::ConstPtr& msg);
  void powerEventCB(const kobuki_comms::PowerSystemEvent::ConstPtr& msg);
  void inputEventCB(const kobuki_comms::DigitalInputEvent::ConstPtr& msg);
  void robotEventCB(const kobuki_comms::RobotStateEvent::ConstPtr& msg);
  void timerEventCB(const ros::TimerEvent& event);
  void robotStatusCB(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);
  void diagnosticsCB(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);

  /*********************
  ** Other methods
  **********************/
  void testLeds();
  void testSounds();
  void move(double v, double w, double t = 0.0);
  bool saveResults(Robot* robot);
};

}  // namespace kobuki_factory_test

#endif /* KOBUKI_FACTORY_TEST_QNODE_HPP_ */
