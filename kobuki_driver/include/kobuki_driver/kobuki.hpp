/**
 * @file include/kobuki_driver/kobuki.hpp
 *
 * @brief Device driver core interface.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
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
  ThreeAxisGyro::Data getRawInertiaData() const { return three_axis_gyro.data; }

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

  /*********************
  ** Debugging
  **********************/
  void printSigSlotConnections() const;

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
  ** Acceleration Limiter
  **********************/
  AccelerationLimiter acceleration_limiter;

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
  UniqueDeviceID unique_device_id; // requestable
  ThreeAxisGyro three_axis_gyro;

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
