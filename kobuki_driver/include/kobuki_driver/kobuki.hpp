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
#include <ecl/config.hpp>
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
#include "macros.hpp"

/*****************************************************************************
** Extern Templates
*****************************************************************************/

#ifdef ECL_IS_WIN32
  /* Help windows create common instances of sigslots across kobuki dll
   * and end user program (otherwise it creates two separate variables!) */
  EXP_TEMPLATE template class kobuki_PUBLIC ecl::SigSlotsManager<>;
  EXP_TEMPLATE template class kobuki_PUBLIC ecl::SigSlotsManager<const kobuki::VersionInfo&>;
  EXP_TEMPLATE template class kobuki_PUBLIC ecl::SigSlotsManager<const std::string&>;
  EXP_TEMPLATE template class kobuki_PUBLIC ecl::SigSlotsManager<kobuki::Command::Buffer&>;
  EXP_TEMPLATE template class kobuki_PUBLIC ecl::SigSlotsManager<kobuki::PacketFinderBase::BufferType&>;
#endif

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
class kobuki_PUBLIC Kobuki
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

  /******************************************
  ** Packet Processing
  *******************************************/
  void spin();
  void fixPayload(ecl::PushAndPop<unsigned char> & byteStream);

  /******************************************
  ** Getters - Data Protection
  *******************************************/
  void lockDataAccess();
  void unlockDataAccess();

  /******************************************
  ** Getters - User Friendly Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
  ecl::Angle<double> getHeading() const;
  double getAngularVelocity() const;
  VersionInfo versionInfo() const { return VersionInfo(firmware.data.version, hardware.data.version, unique_device_id.data.udid0, unique_device_id.data.udid1, unique_device_id.data.udid2); }
  Battery batteryStatus() const { return Battery(core_sensors.data.battery, core_sensors.data.charger); }

  /******************************************
  ** Getters - Raw Data Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
  CoreSensors::Data getCoreSensorData() const { return core_sensors.data; }
  DockIR::Data getDockIRData() const { return dock_ir.data; }
  Cliff::Data getCliffData() const { return cliff.data; }
  Current::Data getCurrentData() const { return current.data; }
  Inertia::Data getInertiaData() const { return inertia.data; }
  GpInput::Data getGpInputData() const { return gp_input.data; }
  ThreeAxisGyro::Data getRawInertiaData() const { return three_axis_gyro.data; }
  ControllerInfo::Data getControllerInfoData() const { return controller_info.data; }

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
  bool setControllerGain(const unsigned char &type, const unsigned int &p_gain,
                         const unsigned int &i_gain, const unsigned int &d_gain);
  bool getControllerGain();

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
  ** Inertia
  **********************/
  double heading_offset;

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
  ControllerInfo controller_info; // requestable

  ecl::Serial serial;
  PacketFinder packet_finder;
  PacketFinder::BufferType data_buffer;
  bool is_alive; // used as a flag set by the data stream watchdog

  int version_info_reminder;
  int controller_info_reminder;

  /*********************
  ** Commands
  **********************/
  void sendBaseControlCommand();
  void sendCommand(Command command);
  ecl::Mutex command_mutex; // protection against the user calling the command functions from multiple threads
  // data_mutex is protection against reading and writing data structures simultaneously as well as
  // ensuring multiple get*** calls are synchronised to the same data update
  // refer to https://github.com/yujinrobot/kobuki/issues/240
  ecl::Mutex data_mutex;
  Command kobuki_command; // used to maintain some state about the command history
  Command::Buffer command_buffer;
  std::vector<short> velocity_commands_debug;

  /*********************
  ** Events
  **********************/
  EventManager event_manager;

  /*********************
  ** Logging
  **********************/
  std::vector<std::string> log(std::string msg) { return log("", "", msg); } 
  std::vector<std::string> log(std::string level, std::string msg) { return log(level, "", msg); }
  std::vector<std::string> log(std::string level, std::string name, std::string msg) {
    std::vector<std::string> ret;
    if( level != "" ) ret.push_back(level);
    if( name != "" ) ret.push_back(name);
    if( msg != "" ) ret.push_back(msg);
    return ret;
  }

  /*********************
  ** Signals
  **********************/
  ecl::Signal<> sig_stream_data, sig_controller_info;
  ecl::Signal<const VersionInfo&> sig_version_info;
  ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;
  ecl::Signal<const std::vector<std::string>&> sig_named;
  ecl::Signal<Command::Buffer&> sig_raw_data_command; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
  ecl::Signal<PacketFinder::BufferType&> sig_raw_data_stream; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
  ecl::Signal<const std::vector<short>&> sig_raw_control_command;
};

} // namespace kobuki

#endif /* KOBUKI_HPP_ */
