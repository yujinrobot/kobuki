/**
 * @file /include/kobuki/kobuki.hpp
 *
 * @brief Cpp device driver core interface.
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_HPP_
#define KOBUKI_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <iostream>
#include <string>
#include <algorithm>
#include <kobuki_comms/CoreSensors.h>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/time.hpp>
#include <ecl/mobile_robot.hpp>

#include <ecl/exceptions/standard_exception.hpp>
#include "packet_handler/packet_finder.hpp"
#include "parameters.hpp"

#include "core_sensors.hpp"
#include "ir.hpp"
#include "dock_ir.hpp"
#include "modules/inertia.hpp"
#include "cliff.hpp"
#include "current.hpp"
#include "time.hpp"
#include "hw.hpp"
#include "fw.hpp"
#include "eeprom.hpp"
#include "gp_input.hpp"
#include "command.hpp"

#include "simulation.hpp"

#include "led_array.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Definitions
 *****************************************************************************/

union union_sint16
{
  short word;
  unsigned char byte[2];
};

/*****************************************************************************
** Parent Interface
*****************************************************************************/

class PacketFinder : public PacketFinderBase
{
public:
  virtual ~PacketFinder() {}
  bool checkSum();
};

/*****************************************************************************
 ** Interface [Kobuki]
 *****************************************************************************/
/**
 * @brief  The core kobuki driver class.
 *
 * This connects to the outside world via sigslots and get accessors.
 **/
class Kobuki : public ecl::Threadable
{
public:
  Kobuki() :
    last_velocity_left(0.0),
    last_velocity_right(0.0),
    is_connected(false), is_running(false), is_enabled(false),
    tick_to_mm(0.0845813406577f), tick_to_rad(0.00201384144460884f)
  {
  }
  ~Kobuki()
  {
    serial.close();
    is_connected = false;
    is_running = false;
    is_enabled = false;
  }

  /*********************
   ** Configuration
   **********************/
  void runnable();
  void init(Parameters &parameters) throw (ecl::StandardException);
  bool connected() const
  {
    return is_connected;
  }
  bool isEnabled() const
  {
    return is_enabled;
  }
  bool enable();
  bool disable();
  void close();

  ecl::Angle<double> getHeading() const;

  void getCoreSensorData(kobuki_comms::CoreSensors&);
  void getIRData(kobuki_comms::IR&);
  void getDockIRData(kobuki_comms::DockIR&);
  void getCliffData(kobuki_comms::Cliff&);
  void getCurrentData(kobuki_comms::Current&);
  void getHWData(kobuki_comms::HW&);
  void getFWData(kobuki_comms::FW&);
  void getTimeData(kobuki_comms::Time&);
  void getEEPROMData(kobuki_comms::EEPROM&);
  void getGpInputData(kobuki_comms::GpInput&);

  /*********************
  ** Feedback
  **********************/
  void getWheelJointStates(double &wheel_left_angle, double &wheel_left_angle_rate,
                            double &wheel_right_angle, double &wheel_right_angle_rate);
  void updateOdometry(ecl::Pose2D<double> &pose_update,
                      ecl::linear_algebra::Vector3d &pose_update_rates);
  /*********************
  ** Soft Commands
  **********************/
  void resetOdometry();

  /*********************
  ** Hard Commands
  **********************/
  void toggleLed(const enum LedNumber &number, const enum LedColour &colour);
  void setBaseControlCommand(double, double);
  void sendBaseControlCommand();
  void sendCommand(Command &command);

private:
  ecl::StopWatch stopwatch;

  unsigned short last_timestamp;
  double last_velocity_left, last_velocity_right;
  double last_diff_time;

  unsigned short last_tick_left, last_tick_right;
  double last_rad_left, last_rad_right;
  double last_mm_left, last_mm_right;

  short v, w;
  short radius;
  short speed;
  double bias; //wheelbase, wheel_to_wheel, in [m]
  double wheel_radius;
  int imu_heading_offset;

  std::string device_type;
  std::string protocol_version;
  bool is_connected; // True if there's a serial/usb connection open.
  bool is_running;
  bool is_enabled;

  unsigned int count;
  const double tick_to_mm, tick_to_rad;

  ecl::Serial serial;

  DefaultData kobuki_default;
  IRData kobuki_ir;
  DockIRData kobuki_dock_ir;
  Inertia kobuki_inertia;
  CliffData kobuki_cliff;
  CurrentData kobuki_current;
  TimeData kobuki_time;
  HWData kobuki_hw;
  FWData kobuki_fw;
  EEPROMData kobuki_eeprom;
  GpInputData kobuki_gp_input;
  SimulationData kobuki_sim;

  Command kobuki_command;

  PacketFinder packet_finder;
  PacketFinder::BufferType data_buffer;
  ecl::PushAndPop<unsigned char> command_buffer;
  ecl::Signal<> sig_wheel_state, sig_core_sensors;
  ecl::Signal<> sig_ir, sig_dock_ir, sig_inertia, sig_cliff, sig_current, sig_magnet, sig_hw, sig_fw,
                sig_time, sig_eeprom, sig_gp_input;

  ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;

  boost::shared_ptr<ecl::DifferentialDrive::Kinematics> kinematics;
  bool simulation;
};

} // namespace kobuki

#endif /* KOBUKI_HPP_ */
