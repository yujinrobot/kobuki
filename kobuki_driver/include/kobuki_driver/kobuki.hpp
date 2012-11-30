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

#include <string>
#include <iomanip>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "version_info.hpp"
#include "parameters.hpp"
#include "event_manager.hpp"
#include "command.hpp"
#include "modules.hpp"
#include "packets.hpp"
#include "packet_handler/packet_finder.hpp"

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
  Kobuki();
  ~Kobuki();

  /*********************
   ** Configuration
   **********************/
  void init(Parameters &parameters) throw (ecl::StandardException);
  bool isAlive() const { return is_alive; } /**< Whether the connection to the robot is alive and currently streaming. **/
  bool isShutdown() const { return shutdown_requested; } /**< Whether the worker thread is alive or not. **/
  bool isEnabled() const { return is_enabled; } /**< Whether the motor power is enabled or disabled. **/
  bool enable(); /**< Enable power to the motors. **/
  bool disable(); /**< Disable power to the motors. **/
  void shutdown() { shutdown_requested = true; } /**< Gently terminate the worker thread. **/
  void spin();

  /******************************************
  ** User Friendly Api
  *******************************************/
  ecl::Angle<double> getHeading() const;
  double getAngularVelocity() const;
  VersionInfo versionInfo() const { return VersionInfo(firmware.data.version, hardware.data.version, unique_device_id.data.udid0, unique_device_id.data.udid1, unique_device_id.data.udid2); }
  Battery batteryStatus() const { return Battery(core_sensors.data.battery, core_sensors.data.charger); }

  /******************************************
  ** Raw Data Api
  *******************************************/
  CoreSensors::Data getCoreSensorData() const { return core_sensors.data; }
  DockIR::Data getDockIRData() const { return dock_ir.data; }
  Cliff::Data getCliffData() const { return cliff.data; }
  Current::Data getCurrentData() const { return current.data; }
  Inertia::Data getInertiaData() const { return inertia.data; }
  GpInput::Data getGpInputData() const { return gp_input.data; }

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
  void setBaseControl(const double &linear_velocity, const double &angular_velocity);
  void setLed(const enum LedNumber &number, const enum LedColour &colour);
  void setDigitalOutput(const DigitalOutput &digital_output);
  void setExternalPower(const DigitalOutput &digital_output);
  void playSoundSequence(const enum SoundSequences &number);

private:
  /*********************
  ** Thread
  **********************/
  ecl::Thread thread;
  bool shutdown_requested; // helper to shutdown the worker thread.

  /*********************
  ** Odometry
  **********************/
  DiffDrive diff_drive;
  bool is_enabled;

  /*********************
  ** Driver Paramters
  **********************/
  Parameters parameters;
  bool is_connected;

  /*********************
  ** Gate Keeper / High Acceleration Smoother / Limiter
  **********************/
  GateKeeper gate_keeper;

  /*********************
  ** Packet Handling
  **********************/
  CoreSensors core_sensors;
  Inertia inertia;
  DockIR dock_ir;
  Cliff cliff;
  Current current;
  GpInput gp_input;
  Hardware hardware; // requestable
  Firmware firmware; // requestable
  UniqueDeviceID unique_device_id;

  std::string protocol_version;
  ecl::Serial serial;
  PacketFinder packet_finder;
  PacketFinder::BufferType data_buffer;
  bool is_alive; // used as a flag set by the data stream watchdog

  int version_info_reminder;

  /*********************
  ** Commands
  **********************/
  void sendBaseControlCommand();
  void sendCommand(Command command);
  ecl::Mutex command_mutex; // protection against the user calling the command functions from multiple threads
  Command kobuki_command; // used to maintain some state about the command history
  Command::Buffer command_buffer;

  /*********************
  ** Events
  **********************/
  EventManager event_manager;

  /*********************
  ** Signals
  **********************/
  ecl::Signal<> sig_stream_data;
  ecl::Signal<const VersionInfo&> sig_version_info;
  ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;
  ecl::Signal<Command::Buffer&> sig_raw_data_command; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
  ecl::Signal<PacketFinder::BufferType&> sig_raw_data_stream; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
};

} // namespace kobuki

#endif /* KOBUKI_HPP_ */
