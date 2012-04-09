/**
 * @file /cruizcore/include/cruizcore/cruizcore.hpp
 *
 * @brief Cpp interface for a cruizcore gyro device driver.
 *
 * @date 20/08/2010
 **/
/*****************************************************************************
<<<<<<< HEAD
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
#include <kobuki_comms/SensorData.h>
#include <device_comms/JointState.h>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/time.hpp>

#include <ecl/exceptions/standard_exception.hpp>
#include "packet_finder.hpp"
#include "parameters.hpp"

// [ version 2 ]
#include <kobuki_comms/Header.h>
#include "default.hpp"
#include "ir.hpp"
#include "dock_ir.hpp"
#include "inertia.hpp"
#include "cliff.hpp"
#include "current.hpp"
#include "magnet.hpp"
#include "time.hpp"
#include "hw.hpp"
#include "fw.hpp"
#include "st_gyro.hpp"
#include "eeprom.hpp"
#include "gp_input.hpp"
#include "command.hpp"

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
  //const device_comms::ns::Gyro& data() const { return gyro_data; }
  bool run();
  bool stop();
  void close();
  void getSensorData(kobuki_comms::SensorData&);
  void getIRData(kobuki_comms::IR&);
  void getDockIRData(kobuki_comms::DockIR&);
  void getInertiaData(kobuki_comms::Inertia&);
  void getCliffData(kobuki_comms::Cliff&);
  void getCurrentData(kobuki_comms::Current&);
  void getMagnetData(kobuki_comms::Magnet&);
  void getHWData(kobuki_comms::HW&);
  void getFWData(kobuki_comms::FW&);
  void getTimeData(kobuki_comms::Time&);
  void getStGyroData(kobuki_comms::StGyro&);
  void getEEPROMData(kobuki_comms::EEPROM&);
  void getGpInputData(kobuki_comms::GpInput&);

  void updateOdometry(double &wheel_left_position, double &wheel_left_velocity,
                      double &wheel_right_position, double &wheel_right_velocity);
  void getJointState(device_comms::JointState&);
  void setCommand(double, double);
  void sendCommand();
  void sendCommand(const kobuki_comms::CommandConstPtr &data);
  void pubtime(const char *);

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

  std::string device_type;
  std::string protocol_version;
  bool is_connected; // True if there's a serial/usb connection open.
  //device_comms::ns::Gyro gyro_data;
  bool is_running;
  bool is_enabled;

  unsigned int count;
  const double tick_to_mm, tick_to_rad;

  ecl::Serial serial;

  DefaultData kobuki_default;
  IRData kobuki_ir;
  DockIRData kobuki_dock_ir;
  InertiaData kobuki_inertia;
  CliffData kobuki_cliff;
  CurrentData kobuki_current;
  MagnetData kobuki_magnet;
  TimeData kobuki_time;
  HWData kobuki_hw;
  FWData kobuki_fw;
  StGyroData kobuki_st_gyro;
  EEPROMData kobuki_eeprom;
  GpInputData kobuki_gp_input;

  CommandData kobuki_command;

  PacketFinder packet_finder;
  PacketFinder::BufferType data_buffer;
  ecl::PushAndPop<unsigned char> command_buffer;

  ecl::Signal<> sig_wheel_state, sig_sensor_data;
  ecl::Signal<> sig_ir, sig_dock_ir, sig_inertia, sig_cliff, sig_current, sig_magnet, sig_hw, sig_fw,
                sig_time, sig_st_gyro, sig_eeprom, sig_gp_input;

  ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;

  std::set<unsigned char> sig_index;
};

} // namespace cruizcore

#endif /* KOBUKI_HPP_ */
