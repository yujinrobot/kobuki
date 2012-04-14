/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
#include <boost/shared_ptr.hpp>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/time.hpp>
#include <ecl/mobile_robot.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "packet_handler/packet_finder.hpp"
#include "parameters.hpp"
#include "modules/cliff.hpp"
#include "modules/core_sensors.hpp"
#include "modules/current.hpp"
#include "modules/gp_input.hpp"
#include "modules/inertia.hpp"
#include "modules/dock_ir.hpp"
#include "modules/firmware.hpp"
#include "modules/hardware.hpp"
#include "command.hpp"
#include "simulation.hpp"
#include "led_array.hpp"
#include "version_info.hpp"

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
class Kobuki
{
public:
  Kobuki() :
    last_velocity_left(0.0),
    last_velocity_right(0.0),
    is_connected(false), shutdown_requested(false), is_enabled(false),
    tick_to_mm(0.0845813406577f), tick_to_rad(0.00201384144460884f)
  {
  }
  ~Kobuki();

  /*********************
   ** Configuration
   **********************/
  void spin();
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

  /******************************************
  ** User Friendly Api
  *******************************************/
  ecl::Angle<double> getHeading() const;
  double getAngularVelocity() const;
  VersionInfo versionInfo() const { return VersionInfo(firmware.data.version, hardware.data.version); }

  /******************************************
  ** Raw Data Api
  *******************************************/
  void getCoreSensorData(CoreSensors::Data&) const;
  void getDockIRData(DockIR::Data&) const;
  void getCliffData(Cliff::Data&) const;
  void getCurrentData(Current::Data&) const;
  void getGpInputData(GpInput::Data&) const;

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
  void sendCommand(Command command);

private:
  ecl::Thread thread;

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

  std::string protocol_version;
  bool is_connected; // True if there's a serial/usb connection open.
  bool shutdown_requested; // helper to shutdown the worker thread.
  bool is_enabled;

  const double tick_to_mm, tick_to_rad;

  // Streamed Data
  CoreSensors core_sensors;
  Inertia inertia;
  DockIR dock_ir;
  Cliff cliff;
  Current current;
  GpInput gp_input;
  // Request Services
  Hardware hardware;
  Firmware firmware;

  Simulation simulation;
  Command kobuki_command;

  ecl::Serial serial;
  PacketFinder packet_finder;
  PacketFinder::BufferType data_buffer;
  ecl::PushAndPop<unsigned char> command_buffer;

  ecl::Signal<> sig_stream_data;
  ecl::Signal<const VersionInfo&> sig_version_info;
  ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;

  boost::shared_ptr<ecl::DifferentialDrive::Kinematics> kinematics;
  bool is_simulation;
};

} // namespace kobuki

#endif /* KOBUKI_HPP_ */
