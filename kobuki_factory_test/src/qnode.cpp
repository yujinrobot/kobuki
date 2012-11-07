/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <math.h>
#include <sstream>
#include <cstdarg>

#include <QMessageBox>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <kobuki_comms/Led.h>
#include <kobuki_comms/Sound.h>
#include <kobuki_comms/DigitalOutput.h>

#include "../include/kobuki_factory_test/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_factory_test {

/*****************************************************************************
** Constants
*****************************************************************************/

#define TEST_MOTORS_V    0.2        // lin. speed, in m/s
#define TEST_MOTORS_W   (M_PI/3.0)  // ang. speed, in rad/s
#define TEST_MOTORS_D    0.6        // distance, in m
#define TEST_MOTORS_A   (2.0*M_PI)  // turning, in rad
#define TEST_BUMPERS_V   0.1
#define TEST_BUMPERS_W  (M_PI/6.0)

/*****************************************************************************
** Implementation
*****************************************************************************/

// Define a postfix increment operator for QNode::EvalStep
inline QNode::EvalStep operator++(QNode::EvalStep& es, int)
{
  return es = (QNode::EvalStep)(es + 1);
}


QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv),
  under_test(NULL)
  {}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void QNode::versionInfoCB(const kobuki_comms::VersionInfo::ConstPtr& msg) {
  if ((under_test == NULL) || (under_test->device_ok[Robot::V_INFO] == true))
    return;

  under_test->device_val[Robot::V_INFO] |= msg->firmware; under_test->device_val[Robot::V_INFO] <<= 16;
  under_test->device_val[Robot::V_INFO] |= msg->hardware; under_test->device_val[Robot::V_INFO] <<= 32;
  under_test->device_val[Robot::V_INFO] |= msg->software;
  under_test->device_ok[Robot::V_INFO] = true;

  log(Info, "Hardware/firmware/software version: %s", under_test->version_nb().c_str());
}

void QNode::sensorsCoreCB(const kobuki_comms::SensorState::ConstPtr& msg) {
  if (under_test == NULL)
    return;

  if ((current_step >= TEST_MOTORS_FORWARD) && (current_step <= TEST_MOTORS_COUNTERCW)) {
    under_test->device_val[Robot::MOTOR_L] =
        std::max((uint8_t)under_test->device_val[Robot::MOTOR_L], msg->current[0]);
    under_test->device_val[Robot::MOTOR_R] =
        std::max((uint8_t)under_test->device_val[Robot::MOTOR_R], msg->current[1]);
  }
}

void QNode::dockBeaconCB(const kobuki_comms::DockInfraRed::ConstPtr& msg) {
  if ((under_test == NULL) || (under_test->ir_dock_ok() == true))
    return;

  if (msg->data[0] > 0) {
    under_test->device_val[Robot::IR_DOCK_L] = msg->data[0];
    under_test->device_ok[Robot::IR_DOCK_L] = true;
  }

  if (msg->data[1] > 0) {
    under_test->device_val[Robot::IR_DOCK_C] = msg->data[1];
    under_test->device_ok[Robot::IR_DOCK_C] = true;
  }

  if (msg->data[2] > 0) {
    under_test->device_val[Robot::IR_DOCK_R] = msg->data[2];
    under_test->device_ok[Robot::IR_DOCK_R] = true;
  }

  if (under_test->ir_dock_ok() == true)
    log(Info, "Docking ir sensor evaluation completed: %d/%d/%d",
                under_test->device_val[Robot::IR_DOCK_L],
                under_test->device_val[Robot::IR_DOCK_C],
                under_test->device_val[Robot::IR_DOCK_R]);

//  under_test->device_val[Robot::IR_DOCK_L] = std::max(under_test->device_val[Robot::IR_DOCK_L], (int)msg->data[0]);
//  under_test->device_val[Robot::IR_DOCK_C] = std::max(under_test->device_val[Robot::IR_DOCK_C], (int)msg->data[1]);
//  under_test->device_val[Robot::IR_DOCK_R] = std::max(under_test->device_val[Robot::IR_DOCK_R], (int)msg->data[2]);

  }

void QNode::gyroscopeCB(const sensor_msgs::Imu::ConstPtr& msg) {
  ;
}

void QNode::buttonEventCB(const kobuki_comms::ButtonEvent::ConstPtr& msg) {
  if (under_test == NULL)
    return;

  if ((current_step == TEST_LEDS) && (msg->state  == kobuki_comms::ButtonEvent::PRESSED)) {
    // We are currently evaluating the LEDs; if tester press first button for ok or third otherwise
    if (msg->button == kobuki_comms::ButtonEvent::Button0) {
      under_test->device_ok[Robot::LED_1] = under_test->device_ok[Robot::LED_2] = true;
      log(Info, "LEDs evaluation completed");
      log(Info, "Raise and lower the robot three times");
      current_step++;
    }
    else if (msg->button == kobuki_comms::ButtonEvent::Button2) {
      under_test->device_ok[Robot::LED_1] = under_test->device_ok[Robot::LED_2] = false;
      log(Warn, "LEDs didn't pass the test");  // TODO  should we cancel eval?
      current_step++;
    }
    return;
  }

  // Buttons evaluation is completed
  if (under_test->buttons_ok() == true)
    return;

  if ((current_step < BUTTON_0_PRESSED) || (current_step > BUTTON_2_RELEASED)) {
    // It's not time to evaluate buttons; assume it's an accidental hit
    log(Debug, "Button %d accidental hit; ignoring", msg->button);
    return;
  }

  // Note that we assume that buttons are 0, 1 and 2, and states are 1 (pressed) and 0 (released)
  uint8_t expected_button = (current_step - BUTTON_0_PRESSED)/2;
  uint8_t expected_action = (current_step - BUTTON_0_PRESSED)%2 ^ 1;

//  std::cout << "\n\n" << (int)expected_button << "   " << (int)expected_action << "   " << current_step << "    " << dev << "    " << "\n\n";

  if ((msg->button == expected_button) && (msg->state == expected_action)) {
    Robot::Device dev = (Robot::Device)(Robot::BUTTON_0 + msg->button);

    log(Info, "Button %d %s, as expected", msg->button, msg->state?"pressed":"released");

    if (msg->state == kobuki_comms::ButtonEvent::RELEASED)
      under_test->device_ok[dev] = true;

    if (current_step == BUTTON_2_RELEASED)
      log(Info, "Buttons evaluation completed");

    current_step++;
  }
  else
    log(Warn, "Unexpected button event: %d %s", msg->button, msg->state?"pressed":"released");
}

void QNode::bumperEventCB(const kobuki_comms::BumperEvent::ConstPtr& msg) {
  if ((under_test == NULL) || (under_test->bumpers_ok() == true))
    return;

  if ((current_step < CENTER_BUMPER_PRESSED) || (current_step > LEFT_BUMPER_RELEASED)) {
    // it's not time to evaluate bumpers; assume it's an accidental hit
    log(Debug, "Bumper %d accidental hit; ignoring", msg->bumper);
    return;
  }

  // Note that we assume that bumpers are 0, 1 and 2, and states are 1 (pressed) and 0 (released)
  int expected_bumper = ((current_step - CENTER_BUMPER_PRESSED)/3 + 1) % 3;
  int expected_action = ((current_step - CENTER_BUMPER_PRESSED)%3 ^ 1);

  if ((msg->bumper == expected_bumper) && (msg->state == expected_action)) {
//    Robot::Device dev = (Robot::Device)(Robot::BUMPER_L + msg->bumper);
//    std::cout << "\n" << (int)expected_bumper << "   " << (int)expected_action << "   " << (int)current_step << "    " << (int)dev << "    " << "\n";

    log(Info, "Bumper %d %s, as expected", msg->bumper, msg->state?"pressed":"released");

    if (msg->state == kobuki_comms::BumperEvent::PRESSED) {
      move(-TEST_BUMPERS_V, 0.0, 1.5);
      current_step++;
    }
    else
      under_test->device_ok[Robot::BUMPER_L + msg->bumper] = true;

    if (current_step == LEFT_BUMPER_RELEASED)
      log(Info, "Bumper evaluation completed");
  }
  else
    log(Warn, "Unexpected bumper event: %d %s", msg->bumper, msg->state?"pressed":"released");
}

void QNode::wDropEventCB(const kobuki_comms::WheelDropEvent::ConstPtr& msg) {
  if ((under_test == NULL) || (under_test->w_drop_ok() == true))
    return;

  Robot::Device dev =
      (msg->wheel == kobuki_comms::WheelDropEvent::LEFT)?Robot::W_DROP_L:Robot::W_DROP_R;
  if (under_test->device_ok[dev] == true)
    return;

  if (((msg->state == kobuki_comms::WheelDropEvent::DROPPED) &&
       (under_test->device_val[dev] % 2 == 0)) ||
      ((msg->state == kobuki_comms::WheelDropEvent::RAISED) &&
       (under_test->device_val[dev] % 2 == 1))) {
    log(Info, "%s wheel %s, as expected", msg->wheel?"Right":"Left", msg->state?"dropped":"raised");
    under_test->device_val[dev]++;

    if (under_test->device_val[dev] == 6) {
      log(Info, "%s wheel drop evaluation completed", msg->wheel?"Right":"Left");
      under_test->device_ok[dev] = true;
    }
  }
  else
    log(Warn, "Unexpected wheel drop event: %d, %d", msg->wheel, msg->state);
}

void QNode::cliffEventCB(const kobuki_comms::CliffEvent::ConstPtr& msg) {
  if ((under_test == NULL) || (under_test->cliffs_ok() == true))
    return;

  Robot::Device dev = (msg->sensor == kobuki_comms::CliffEvent::LEFT) ?Robot::CLIFF_L:
                      (msg->sensor == kobuki_comms::CliffEvent::RIGHT)?Robot::CLIFF_R:
                                                                       Robot::CLIFF_C;
  if (under_test->device_ok[dev] == true)
    return;

  if (((msg->state == kobuki_comms::CliffEvent::CLIFF) &&
       (under_test->device_val[dev] % 2 == 0)) ||
      ((msg->state == kobuki_comms::CliffEvent::FLOOR) &&
       (under_test->device_val[dev] % 2 == 1))) {
    log(Info, "%s cliff sensor reports %s, as expected",
         dev == Robot::CLIFF_R?"Right":dev == Robot::CLIFF_C?"Center":"Left",
        msg->state?"cliff":"no cliff");
    under_test->device_val[dev]++;

    if (under_test->device_val[dev] == 6) {
      log(Info, "%s cliff sensor evaluation completed",
           dev == Robot::CLIFF_R?"Right":dev == Robot::CLIFF_C?"Center":"Left");
      under_test->device_ok[dev] = true;
    }
  }
  else
    log(Warn, "Unexpected cliff sensor event: %d, %d", msg->sensor, msg->state);
}

void QNode::powerEventCB(const kobuki_comms::PowerSystemEvent::ConstPtr& msg) {
  if ((under_test == NULL) || (under_test->pwr_src_ok() == true))
    return;

  switch (current_step) {
    case DC_JACK_PLUGGED:
      if (msg->event == kobuki_comms::PowerSystemEvent::PLUGGED_TO_ADAPTER) {
        log(Info, "Adapter %s, as expected", msg->event?"plugged":"unplugged");
        under_test->device_val[Robot::PWR_JACK]++;
        current_step = DC_JACK_UNPLUGGED;
      }
      else
        log(Warn, "Unexpected power event: %d", msg->event);
      break;
    case DC_JACK_UNPLUGGED:
      if (msg->event == kobuki_comms::PowerSystemEvent::UNPLUGGED) {
        log(Info, "Adapter %s, as expected", msg->event?"plugged":"unplugged");
        under_test->device_val[Robot::PWR_JACK]++;

        if (under_test->device_val[Robot::PWR_JACK] == 6) {
          log(Info, "Adapter plugging evaluation completed; connect robot to docking base three times");
          under_test->device_ok[Robot::PWR_JACK] = true;

          current_step = DOCKING_PLUGGED;
        }
        else
          current_step = DC_JACK_PLUGGED;
      }
      else
        log(Warn, "Unexpected power event: %d", msg->event);
      break;
    case DOCKING_PLUGGED:
      if (msg->event == kobuki_comms::PowerSystemEvent::PLUGGED_TO_DOCKBASE) {
        log(Info, "Docking base %s, as expected", msg->event?"plugged":"unplugged");
        under_test->device_val[Robot::PWR_DOCK]++;
        current_step = DOCKING_UNPLUGGED;
      }
      else
        log(Warn, "Unexpected power event: %d", msg->event);
      break;
    case DOCKING_UNPLUGGED:
      if (msg->event == kobuki_comms::PowerSystemEvent::UNPLUGGED) {
        log(Info, "Docking base %s, as expected", msg->event?"plugged":"unplugged");
        under_test->device_val[Robot::PWR_DOCK]++;

        if (under_test->device_val[Robot::PWR_DOCK] == 6) {
          log(Info, "Adapter plugging evaluation completed");
          under_test->device_ok[Robot::PWR_DOCK] = true;

//          /*  TODO NEXT STATE */current_step = BUTTON_0_PRESSED;
        }
        else
          current_step = DOCKING_PLUGGED;
      }
      else
        log(Warn, "Unexpected power event: %d", msg->event);
      break;
    default:
      // it's not time to evaluate power sources; it can be just an irrelevant event but
      // also the tester can not be following the protocol or there's an error in the code
      if ((msg->event != kobuki_comms::PowerSystemEvent::CHARGE_COMPLETED) &&
          (msg->event != kobuki_comms::PowerSystemEvent::BATTERY_LOW)      &&
          (msg->event != kobuki_comms::PowerSystemEvent::BATTERY_CRITICAL))
      log(Warn, "Unexpected power event: %d", msg->event);
      break;
  }

   /*
  if (((msg->event == kobuki_comms::PowerSystemEvent::PLUGGED_TO_ADAPTER) &&
       (under_test->power[0] % 2 == 0)) ||
      ((msg->event == kobuki_comms::PowerSystemEvent::UNPLUGGED) &&
       (under_test->power[0] % 2 == 1))) {
    log(Info, "Adapter %s, as expected", msg->event?"plugged":"unplugged");
    under_test->power[0]++;
  }
  else
    log(Warn, "Unexpected power event: %d", msg->event);

  if (under_test->power[0] == 6) {
    log(Info, "Adapter plugging evaluation completed");
    under_test->power_ok = true;
  }*/
}

void QNode::inputEventCB(const kobuki_comms::DigitalInputEvent::ConstPtr& msg) {
  ;
}

void QNode::diagnosticsCB(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
  if (under_test == NULL)
    return;

  std::stringstream diagnostics;

  for (unsigned int i = 0; i < msg->status.size(); i++) {
    diagnostics << "Device: "  <<      msg->status[i].name << std::endl;
    diagnostics << "Level: "   << (int)msg->status[i].level << std::endl;  // it's a char!
    diagnostics << "Message: " <<      msg->status[i].message << std::endl;
    for (unsigned int j = 0; j < msg->status[i].values.size(); j++) {
      diagnostics << "   " << msg->status[i].values[j].key
                  << ": "  << msg->status[i].values[j].value << std::endl;
    }
  }

  under_test->diagnostics = diagnostics.str();
}

void QNode::robotStatusCB(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg) {
  if ((under_test == NULL) || (under_test->state == Robot::OK))
    return;

  under_test->state = (Robot::State)msg->level;

  if (msg->level == diagnostic_msgs::DiagnosticStatus::OK) {
    log(Info, "Robot top level diagnostics received with OK status");
  }
  else {
    log(Warn, "Robot top level diagnostics received with %s status",
         msg->level == diagnostic_msgs::DiagnosticStatus::WARN?"WARN":"ERROR");
    if (under_test->diagnostics.size() > 0)
      log(Warn, "Full diagnostics:\n%s",  under_test->diagnostics.c_str());
  }
}

void QNode::robotEventCB(const kobuki_comms::RobotStateEvent::ConstPtr& msg) {
  if (msg->state == kobuki_comms::RobotStateEvent::ONLINE) {
    if (under_test != NULL) {
      log(Warn, "New robot connected while %s is still under evaluation; saving...",
           under_test->serial.c_str());
      saveResults(under_test);
      delete under_test;
    }
    else {
      log(Info, "New robot connected");
    }

    // Create a new robot object
    ros::NodeHandle nh;
    under_test = new Robot(evaluated.size());

    // Resubscribe to version_info to get robot version number (it's a latched topic)
    v_info_sub.shutdown();
    v_info_sub = nh.subscribe("mobile_base/version_info", 1, &QNode::versionInfoCB, this);

    // Point to the first evaluation task
    log(Info, "Connect adapter to robot three times");
  }
  else if (msg->state == kobuki_comms::RobotStateEvent::OFFLINE) {
    if (under_test != NULL) {
      if (under_test->all_ok() == false) {
        // Robot disconnected without finishing the evaluation; assume it failed to pass all tests
        log(Info, "Robot %s disconnected without finishing the evaluation",
             under_test->serial.c_str());
      }
      else {
        log(Info, "Robot %s evaluation succesfully completed", under_test->serial.c_str());
      }

      saveResults(under_test);
      delete under_test;
      under_test = NULL;
    }
    else {
      // This should not happen
      log(Warn, "Robot offline event received, but no robot is under evaluation");
    }
  }
  else {
    // This should not happen
    log(Warn, "Unrecognized robot event received; ignoring");
  }
}

void QNode::timerEventCB(const ros::TimerEvent& event) {
  // jump to next step and stop robot
  current_step++;
  timer_active = false;
  move(0.0, 0.0);
}

void QNode::move(double v, double w, double t) {
  geometry_msgs::Twist vel;

  vel.linear.x  = v;
  vel.angular.z = w;
  cmd_vel_pub.publish(vel);
  if (t > 0.0) {
    timer.stop();
    timer.setPeriod(ros::Duration(t));
    timer.start();
    timer_active = true;
  }
}

void QNode::testLeds() {
  kobuki_comms::Led led;

  log(Info, "You should see both leds blinking in green, orange and red alternatively");
  log(Info, "Press first function button if so or third otherwise");

  Q_EMIT showMessage("LEDs test",
                     "You should see both LEDs blinking in green, orange and red alternatively" \
                     "\nPress first function button if so or third otherwise");
//  QMessageBox::information(NULL, "LEDs test",
//                           "You should see both LEDs blinking in green, orange and red alternatively\n" \
//                           "Press first function button if so or third otherwise", QMessageBox::NoButton);

  for (unsigned int i = 0; i < 3; i++) {
    for (uint8_t c = kobuki_comms::Led::GREEN; c <= kobuki_comms::Led::RED; c++) {
      ros::Duration(0.5).sleep();

      led.value = c;
      led_1_pub.publish(led);
      led_2_pub.publish(led);

      ros::Duration(0.5).sleep();

      led.value = kobuki_comms::Led::BLACK;
      led_1_pub.publish(led);
      led_2_pub.publish(led);
    }
  }
}

void QNode::testSounds() {
  kobuki_comms::Led led;

  log(Info, "You should see both leds blinking in green, orange and red alternatively");
  log(Info, "Press first function button if so or third otherwise");

  QMessageBox::information(NULL, "LEDs test",
                           "You should see both LEDs blinking in green, orange and red alternatively\n" \
                           "Press first function button if so or third otherwise", QMessageBox::NoButton);

  for (unsigned int i = 0; i < 3; i++) {
    for (uint8_t c = kobuki_comms::Led::GREEN; c <= kobuki_comms::Led::RED; c++) {
      ros::Duration(0.5).sleep();

      led.value = c;
      led_1_pub.publish(led);
      led_2_pub.publish(led);

      ros::Duration(0.5).sleep();

      led.value = kobuki_comms::Led::BLACK;
      led_1_pub.publish(led);
      led_2_pub.publish(led);
    }
  }
}

bool QNode::saveResults(Robot* robot) {
  log(Error, "TODO save results");
  return true;
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"kobuki_factory_test");
  if ( ! ros::master::check() ) {
      return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;

  // Subscribe to kobuki sensors and publish to its actuators
  v_info_sub  = nh.subscribe("mobile_base/version_info",         10, &QNode::versionInfoCB, this);
  s_core_sub  = nh.subscribe("mobile_base/sensors/core",         10, &QNode::sensorsCoreCB, this);
  beacon_sub  = nh.subscribe("mobile_base/sensors/dock_ir",      10, &QNode::dockBeaconCB,  this);
  gyro_sub    = nh.subscribe("mobile_base/sensors/imu_data",     10, &QNode::gyroscopeCB,   this);
  button_sub  = nh.subscribe("mobile_base/events/button",        10, &QNode::buttonEventCB, this);
  bumper_sub  = nh.subscribe("mobile_base/events/bumper",        10, &QNode::bumperEventCB, this);
  w_drop_sub  = nh.subscribe("mobile_base/events/wheel_drop",    10, &QNode::wDropEventCB,  this);
  cliff_sub   = nh.subscribe("mobile_base/events/cliff",         10, &QNode::cliffEventCB,  this);
  power_sub   = nh.subscribe("mobile_base/events/power_system",  10, &QNode::powerEventCB,  this);
  input_sub   = nh.subscribe("mobile_base/events/digital_input", 10, &QNode::inputEventCB,  this);
  robot_sub   = nh.subscribe("mobile_base/events/robot_state",   10, &QNode::robotEventCB,  this);
  state_sub   = nh.subscribe("diagnostics_toplevel_state",       10, &QNode::robotStatusCB, this);
  diags_sub   = nh.subscribe("diagnostics",                      10, &QNode::diagnosticsCB, this);

  cmd_vel_pub = nh.advertise <geometry_msgs::Twist>        ("cmd_vel", 1);
  led_1_pub   = nh.advertise <kobuki_comms::Led>           ("mobile_base/commands/led1", 1);
  led_2_pub   = nh.advertise <kobuki_comms::Led>           ("mobile_base/commands/led2", 1);
  sound_pub   = nh.advertise <kobuki_comms::Sound>         ("mobile_base/commands/sound", 1);
  output_pub  = nh.advertise <kobuki_comms::DigitalOutput> ("mobile_base/commands/digital_output", 1);
  ext_pwr_pub = nh.advertise <kobuki_comms::DigitalOutput> ("mobile_base/commands/external_power", 1);

  start();

  current_step = RIGHT_BUMPER_PRESSED;
  current_step = BUTTON_0_PRESSED;
  current_step = DC_JACK_PLUGGED;
  current_step = CLIFF_AND_WHEEL_DROP;
  current_step = BUTTON_0_PRESSED;

  timer_active = false;

  // Create a one-shot timer
  timer = nh.createTimer(ros::Duration(1.0), &QNode::timerEventCB, this, true);
  timer.stop();

  return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
  int count = 0;
  EvalStep previous_step = current_step;

  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
    ++count;

    // No robot under evaluation
    if (under_test == NULL)
      continue;

    // A motion is still on course
    if (timer_active == true)
      continue;

    // Here we centralize the the succession of evaluation states
    switch (current_step) {
      case DC_JACK_PLUGGED:
      case DC_JACK_UNPLUGGED:
      case DOCKING_PLUGGED:
      case DOCKING_UNPLUGGED:
        if (under_test->pwr_src_ok() == true) {
          current_step = BUTTON_0_PRESSED;
        }
        break;
      case BUTTON_0_PRESSED:
        if (previous_step != current_step)
          log(Info, "Press the three function buttons sequentially");
        break;
      case BUTTON_0_RELEASED:
      case BUTTON_1_PRESSED:
      case BUTTON_1_RELEASED:
      case BUTTON_2_PRESSED:
      case BUTTON_2_RELEASED:
        if (under_test->buttons_ok() == true) {
//          current_step++;
        }
        break;
      case TEST_LEDS:
        if (previous_step != current_step)
          testLeds();
        break;
      case TEST_SOUNDS:
        if (previous_step != current_step)
          testSounds();
        break;
      case CLIFF_AND_WHEEL_DROP:
        if ((under_test->cliffs_ok() == true) && (under_test->w_drop_ok() == true))  {
          current_step = CENTER_BUMPER_PRESSED;
          log(Info, "Place the robot facing a wall");

          // After a while, launch the robot to bump frontally
          ros::Duration(1.5).sleep();
        }
        break;
      case CENTER_BUMPER_PRESSED:
        move(+TEST_BUMPERS_V, 0.0);
        break;
      case CENTER_BUMPER_RELEASED:
        break;
      case POINT_RIGHT_BUMPER:
        move(0.0, +TEST_BUMPERS_W, 1.5);  // +45 deg, block call
//        current_step = RIGHT_BUMPER_PRESSED;
        break;
      case RIGHT_BUMPER_PRESSED:
        move(+TEST_BUMPERS_V, 0.0);
        break;
      case RIGHT_BUMPER_RELEASED:
        break;
      case POINT_LEFT_BUMPER:
        move(0.0, -TEST_BUMPERS_W, 3.0);  // -90 deg, block call
//        current_step = LEFT_BUMPER_PRESSED;
        break;
      case LEFT_BUMPER_PRESSED:
        move(+TEST_BUMPERS_V, 0.0);
        break;
      case LEFT_BUMPER_RELEASED:
        if (under_test->bumpers_ok() == true)  {
          log(Info, "Now the robot will move forward, backward and spin to evaluate motors");

          // Wait while to let the robot put in position
     //     ros::Duration(3.0).sleep();
  //        current_step = TEST_MOTORS_FORWARD;
        }
        break;
      case PREPARE_MOTORS_TEST:
        move(0.0, -TEST_BUMPERS_W, 1.5);  // -45 deg (parallel to wall)
        break;
      case TEST_MOTORS_FORWARD:
        move(+TEST_MOTORS_V, 0.0, TEST_MOTORS_D/TEST_MOTORS_V);
//        current_step++;
        break;
      case TEST_MOTORS_BACKWARD:
      //      ros::Duration(0.25).sleep();
        move(-TEST_MOTORS_V, 0.0, TEST_MOTORS_D/TEST_MOTORS_V);
  //      current_step++;
        break;
      case TEST_MOTORS_CLOCKWISE:
      //      ros::Duration(0.25).sleep();
        move(0.0, -TEST_MOTORS_W, TEST_MOTORS_A/TEST_MOTORS_W);
    //    current_step++;
        break;
      case TEST_MOTORS_COUNTERCW:
      //      ros::Duration(0.25).sleep();
        move(0.0, +TEST_MOTORS_W, TEST_MOTORS_A/TEST_MOTORS_W);
      //  current_step = SOMETHING_MORE;
        break;

      case EVAL_MOTORS_CURRENT:
        if (under_test->motors_ok() == true)
          log(Info, "Motors current evaluation completed (%d, %d)",
               under_test->device_val[Robot::MOTOR_L], under_test->device_val[Robot::MOTOR_R]);
        else
          log(Warn, "Motors current too high! (%d, %d)",
               under_test->device_val[Robot::MOTOR_L], under_test->device_val[Robot::MOTOR_R]);

      //current_step = RIGHT_BUMPER_PRESSED;
        break;/*
      default:
        // it's not time to receive timer events; probably there's an error in the code
        log(Warn, "Unexpected timer event; current step is %d", current_step);
        move(0.0, 0.0);   // stop robot!
        break;
      }
      */
      default:
        // nothing special at this point; we must be evaluating a multi-step device
        log(Warn, "Unknown evaluation step: %d", current_step);
        break;
    }

    previous_step = current_step;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log(const LogLevel &level, const std::string &format, ...) {
  va_list arguments;
  va_start(arguments, format);

  char str[2048];
  vsnprintf(str, 2048, format.c_str(), arguments);

  std::string msg(str);

  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
    case(Debug) : {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case(Info) : {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case(Warn) : {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case(Error) : {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case(Fatal) : {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace kobuki_factory_test
