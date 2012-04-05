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

// [ version 1 ]
#include "data.hpp"
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
 ** Using
 *****************************************************************************/
using ecl::Threadable;
using ecl::Serial;
using ecl::StopWatch;
using ecl::TimeStamp;

union union_sint16
{
  short word;
  unsigned char byte[2];
};

class PacketFinder : public PacketFinderBase
{
public:
  bool checkSum();
};

/*****************************************************************************
 ** Interface [CruizCore]
 *****************************************************************************/
/**
 * @brief  The device driver for a cruizcore gyro.
 *
 * The actual device driver for the cruizcore gyro. This driver simply provides
 * an api from which a program can be written to interact with the device.
 * It does not actually provide the control loop per se.
 *
 * <b>Signals</b>
 *
 * This class accepts a string via the parameter interface which is used as the
 * base namespace for all sigslots that can be connected. These sigslots
 * enable runtime handling and/or debugging of various features.
 * The string id's associated with these connections
 * are listed below where 'ns' is the namespace as mentioned.
 *
 * - 'ns'/raw_data_received : emits a bytearray when a goo packet is received.
 * - 'ns'/raw_data_sent : emits a bytearray when sending a goo command packet.
 * - 'ns'/serial_timeout : simple emit to notify if an expected packet timed out.
 * - 'ns'/invalid_packet : emits a bytearray whenever a mangled packet is received.
 **/
class Kobuki : public Threadable
{
public:
  Kobuki() :
      is_connected(false), is_running(false), is_enabled(false), tick_to_mm(0.0845813406577f), tick_to_rad(
          0.00201384144460884f)
  {
  }
  //Kobuki(Parameters &parameters) throw(ecl::StandardException);
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
  void getData(kobuki_comms::SensorData&);
  void getData2(kobuki_comms::SensorData&);
  void getDefaultData(kobuki_comms::SensorData&);
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

  void getJointState(device_comms::JointState&);
  void setCommand(double, double);
  void sendCommand();
  void sendCommand(const kobuki_comms::CommandConstPtr &data);
  void pubtime(const char *);

private:
  //KobukiMainboardDriver kobuki_receiver;
  StopWatch stopwatch;

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
  //PacketHandler packet_handler;
  //Packet::Buffer buffer;
  //device_comms::ns::Gyro gyro_data;
  bool is_running;
  bool is_enabled;

  unsigned int count;
  const double tick_to_mm, tick_to_rad;

  Serial serial;
  Data data;
  Data2 data2;

  // [ vserion 2 ]
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
  ecl::Signal<> sig_default, sig_ir, sig_dock_ir, sig_inertia, sig_cliff, sig_current, sig_magnet, sig_hw, sig_fw,
                sig_time, sig_st_gyro, sig_eeprom, sig_gp_input;
//		sig_reserved0;
  std::set<unsigned char> sig_index;
};

} // namespace cruizcore

#endif /* KOBUKI_HPP_ */
