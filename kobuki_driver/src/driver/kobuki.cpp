/**
 * @file src/driver/kobuki.cpp
 *
 * @brief Implementation for the kobuki device driver.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <stdexcept>
#include <ecl/math.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/sleep.hpp>
#include <ecl/converters.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/timestamp.hpp>
#include "../../include/kobuki_driver/kobuki.hpp"
#include "../../include/kobuki_driver/packet_handler/payload_headers.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
 ** Implementation [PacketFinder]
 *****************************************************************************/

bool PacketFinder::checkSum()
{
  unsigned int packet_size(buffer.size());
  unsigned char cs(0);
  for (unsigned int i = 2; i < packet_size; i++)
  {
    cs ^= buffer[i];
  }
  return cs ? false : true;
}

/*****************************************************************************
 ** Implementation [Initialisation]
 *****************************************************************************/

Kobuki::Kobuki() :
    shutdown_requested(false)
    , is_enabled(false)
    , is_connected(false)
    , is_alive(false)
    , version_info_reminder(0)
    , controller_info_reminder(0)
    , heading_offset(0.0/0.0)
    , velocity_commands_debug(4, 0)
{
}

/**
 * Shutdown the driver - make sure we wait for the thread to finish.
 */
Kobuki::~Kobuki()
{
  disable();
  shutdown_requested = true; // thread's spin() will catch this and terminate
  thread.join();
  sig_debug.emit("Device: kobuki driver terminated.");
}

void Kobuki::init(Parameters &parameters) throw (ecl::StandardException)
{

  if (!parameters.validate())
  {
    throw ecl::StandardException(LOC, ecl::ConfigurationError, "Kobuki's parameter settings did not validate.");
  }
  this->parameters = parameters;
  std::string sigslots_namespace = parameters.sigslots_namespace;
  event_manager.init(sigslots_namespace);

  // connect signals
  sig_version_info.connect(sigslots_namespace + std::string("/version_info"));
  sig_controller_info.connect(sigslots_namespace + std::string("/controller_info"));
  sig_stream_data.connect(sigslots_namespace + std::string("/stream_data"));
  sig_raw_data_command.connect(sigslots_namespace + std::string("/raw_data_command"));
  sig_raw_data_stream.connect(sigslots_namespace + std::string("/raw_data_stream"));
  sig_raw_control_command.connect(sigslots_namespace + std::string("/raw_control_command"));
  //sig_serial_timeout.connect(sigslots_namespace+std::string("/serial_timeout"));

  sig_debug.connect(sigslots_namespace + std::string("/ros_debug"));
  sig_info.connect(sigslots_namespace + std::string("/ros_info"));
  sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));
  sig_error.connect(sigslots_namespace + std::string("/ros_error"));
  sig_named.connect(sigslots_namespace + std::string("/ros_named"));

  try {
    serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);  // this will throw exceptions - NotFoundError, OpenError
    is_connected = true;
    serial.block(4000); // blocks by default, but just to be clear!
  }
  catch (const ecl::StandardException &e)
  {
    if (e.flag() == ecl::NotFoundError) {
      sig_warn.emit("device does not (yet) available, is the usb connected?."); // not a failure mode.
    } else {
      throw ecl::StandardException(LOC, e);
    }
  }

  ecl::PushAndPop<unsigned char> stx(2, 0);
  ecl::PushAndPop<unsigned char> etx(1);
  stx.push_back(0xaa);
  stx.push_back(0x55);
  packet_finder.configure(sigslots_namespace, stx, etx, 1, 256, 1, true);
  acceleration_limiter.init(parameters.enable_acceleration_limiter);

  // in case the user changed these from the defaults
  Battery::capacity = parameters.battery_capacity;
  Battery::low = parameters.battery_low;
  Battery::dangerous = parameters.battery_dangerous;

  /******************************************
   ** Get Version Info Commands
   *******************************************/
  version_info_reminder = 10;
  sendCommand(Command::GetVersionInfo());

  /******************************************
   ** Get Controller Info Commands
   *******************************************/
  controller_info_reminder = 10;
  sendCommand(Command::GetControllerGain());
  //sig_controller_info.emit(); //emit default gain

  thread.start(&Kobuki::spin, *this);
}

/*****************************************************************************
 ** Implementation [Runtime]
 *****************************************************************************/
/**
 * Usually you should call the getXXX functions from within slot callbacks
 * connected to this driver's signals. This ensures that data is not
 * overwritten inbetween getXXX calls as it all happens in the serial device's
 * reading thread (aye, convoluted - apologies for the multiple robot and multiple
 * developer adhoc hacking over 4-5 years for hasty demos on pre-kobuki robots.
 * This has generated such wonderful spaghetti ;).
 *
 * If instead you just want to poll kobuki, then you should lock and unlock
 * the data access around any getXXX calls.
 */
void Kobuki::lockDataAccess() {
  data_mutex.lock();
}

/**
 * Unlock a previously locked data access privilege.
 * @sa lockDataAccess()
 */
void Kobuki::unlockDataAccess() {
  data_mutex.unlock();
}

/**
 * @brief Performs a scan looking for incoming data packets.
 *
 * Sits on the device waiting for incoming and then parses it, and signals
 * that an update has occured.
 *
 * Or, if in simulation, just loopsback the motor devices.
 */

void Kobuki::spin()
{
  ecl::TimeStamp last_signal_time;
  ecl::Duration timeout(0.1);
  unsigned char buf[256];

  /*********************
   ** Simulation Params
   **********************/

  while (!shutdown_requested)
  {
    /*********************
     ** Checking Connection
     **********************/
    if ( !serial.open() ) {
      try {
        // this will throw exceptions - NotFoundError is the important one, handle it
        serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
        sig_info.emit("device is connected.");
        is_connected = true;
        serial.block(4000); // blocks by default, but just to be clear!
        event_manager.update(is_connected, is_alive);
        version_info_reminder = 10;
        controller_info_reminder = 10;
      }
      catch (const ecl::StandardException &e)
      {
        // windows throws OpenError if not connected
        if (e.flag() == ecl::NotFoundError) {
          sig_info.emit("device does not (yet) available on this port, waiting...");
        } else if (e.flag() == ecl::OpenError) {
          sig_info.emit("device failed to open, waiting... [" + std::string(e.what()) + "]");
        } else {
          // This is bad - some unknown error we're not handling! But at least throw and show what error we came across.
          throw ecl::StandardException(LOC, e);
        }
        ecl::Sleep(5)(); // five seconds
        is_connected = false;
        is_alive = false;
        continue;
      }
    }

    /*********************
     ** Read Incoming
     **********************/
    int n = serial.read((char*)buf, packet_finder.numberOfDataToRead());
    if (n == 0)
    {
      if (is_alive && ((ecl::TimeStamp() - last_signal_time) > timeout))
      {
        is_alive = false;
        version_info_reminder = 10;
        controller_info_reminder = 10;
        sig_debug.emit("Timed out while waiting for incoming bytes.");
      }
      event_manager.update(is_connected, is_alive);
      continue;
    }
    else
    {
      std::ostringstream ostream;
      ostream << "kobuki_node : serial_read(" << n << ")"
        << ", packet_finder.numberOfDataToRead(" << packet_finder.numberOfDataToRead() << ")";
      //sig_debug.emit(ostream.str());
      sig_named.emit(log("debug", "serial", ostream.str()));
      // might be useful to send this to a topic if there is subscribers
    }

    if (packet_finder.update(buf, n)) // this clears packet finder's buffer and transfers important bytes into it
    {
      PacketFinder::BufferType local_buffer;
      packet_finder.getBuffer(local_buffer); // get a reference to packet finder's buffer.
      sig_raw_data_stream.emit(local_buffer);

      packet_finder.getPayload(data_buffer);// get a reference to packet finder's buffer.

      lockDataAccess();
      while (data_buffer.size() > 0)
      {
        //std::cout << "header_id: " << (unsigned int)data_buffer[0] << " | ";
        //std::cout << "length: " << (unsigned int)data_buffer[1] << " | ";
        //std::cout << "remains: " << data_buffer.size() << " | ";
        //std::cout << "local_buffer: " << local_buffer.size() << " | ";
        //std::cout << std::endl;
        switch (data_buffer[0])
        {
          // these come with the streamed feedback
          case Header::CoreSensors:
            if( !core_sensors.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            event_manager.update(core_sensors.data, cliff.data.bottom);
            break;
          case Header::DockInfraRed:
            if( !dock_ir.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            break;
          case Header::Inertia:
            if( !inertia.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }

            // Issue #274: use first imu reading as zero heading; update when reseting odometry
            if (isnan(heading_offset) == true)
              heading_offset = (static_cast<double>(inertia.data.angle) / 100.0) * ecl::pi / 180.0;
            break;
          case Header::Cliff:
            if( !cliff.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            break;
          case Header::Current:
            if( !current.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            break;
          case Header::GpInput:
            if( !gp_input.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            event_manager.update(gp_input.data.digital_input);
            break;
          case Header::ThreeAxisGyro:
            if( !three_axis_gyro.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            break;
          // the rest are only included on request
          case Header::Hardware:
            if( !hardware.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            //sig_version_info.emit(VersionInfo(firmware.data.version, hardware.data.version));
            break;
          case Header::Firmware:
            if( !firmware.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            try
            {
              // Check firmware/driver compatibility; major version must be the same
              int version_match = firmware.check_major_version();
              if (version_match < 0) {
                sig_error.emit("Robot firmware is outdated and needs to be upgraded. Consult how-to on: " \
                               "http://kobuki.yujinrobot.com/documentation/howtos/upgrading-firmware");
                sig_error.emit("Robot firmware version is " + VersionInfo::toString(firmware.data.version)
                             + "; latest version is " + firmware.current_version());
                shutdown_requested = true;
              }
              else if (version_match > 0) {
                sig_error.emit("Driver version isn't not compatible with robot firmware. Please upgrade driver");
                shutdown_requested = true;
              }
              else
              {
                // And minor version don't need to, but just make a suggestion
                version_match = firmware.check_minor_version();
                if (version_match < 0) {
                  sig_warn.emit("Robot firmware is outdated; we suggest you to upgrade it " \
                                "to benefit from the latest features. Consult how-to on: "  \
                                "http://kobuki.yujinrobot.com/documentation/howtos/upgrading-firmware");
                  sig_warn.emit("Robot firmware version is " + VersionInfo::toString(firmware.data.version)
                              + "; latest version is " + firmware.current_version());
                }
                else if (version_match > 0) {
                  // Driver version is outdated; maybe we should also suggest to upgrade it, but this is not a typical case
                }
              }
            }
            catch (std::out_of_range& e)
            {
              // Wrong version hardcoded on firmware; lowest value is 10000
              sig_error.emit(std::string("Invalid firmware version number: ").append(e.what()));
              shutdown_requested = true;
            }
            break;
          case Header::UniqueDeviceID:
            if( !unique_device_id.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            sig_version_info.emit( VersionInfo( firmware.data.version, hardware.data.version
                , unique_device_id.data.udid0, unique_device_id.data.udid1, unique_device_id.data.udid2 ));
            sig_info.emit("Version info - Hardware: " + VersionInfo::toString(hardware.data.version)
                                     + ". Firmware: " + VersionInfo::toString(firmware.data.version));
            version_info_reminder = 0;
            break;
          case Header::ControllerInfo:
            if( !controller_info.deserialise(data_buffer) ) { fixPayload(data_buffer); break; }
            sig_controller_info.emit();
            controller_info_reminder = 0;
            break;
          default: // in the case of unknown or mal-formed sub-payload
            fixPayload(data_buffer);
            break;
        }
      }
      //std::cout << "---" << std::endl;
      unlockDataAccess();

      is_alive = true;
      event_manager.update(is_connected, is_alive);
      last_signal_time.stamp();
      sig_stream_data.emit();
      sendBaseControlCommand(); // send the command packet to mainboard;
      if( version_info_reminder/*--*/ > 0 ) sendCommand(Command::GetVersionInfo());
      if( controller_info_reminder/*--*/ > 0 ) sendCommand(Command::GetControllerGain());
    }
    else
    {
      // watchdog
      if (is_alive && ((ecl::TimeStamp() - last_signal_time) > timeout))
      {
        is_alive = false;
        // do not call here the event manager update, as it generates a spurious offline state
      }
    }
  }
  sig_error.emit("Driver worker thread shutdown!");
}

void Kobuki::fixPayload(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (byteStream.size() < 3 ) { /* minimum size of sub-payload is 3; header_id, length, data */
    sig_named.emit(log("error", "packet", "too small sub-payload detected."));
    byteStream.clear();
  } else {
    std::stringstream ostream;
    unsigned int header_id = static_cast<unsigned int>(byteStream.pop_front());
    unsigned int length = static_cast<unsigned int>(byteStream.pop_front());
    unsigned int remains = byteStream.size();
    unsigned int to_pop;

    ostream << "[" << header_id << "]";
    ostream << "[" << length << "]";

    ostream << "[";
    ostream << std::setfill('0') << std::uppercase;
    ostream << std::hex << std::setw(2) << header_id << " " << std::dec;
    ostream << std::hex << std::setw(2) << length << " " << std::dec;

    if (remains < length) to_pop = remains;
    else                  to_pop = length;

    for (unsigned int i = 0; i < to_pop; i++ ) {
      unsigned int byte = static_cast<unsigned int>(byteStream.pop_front());
      ostream << std::hex << std::setw(2) << byte << " " << std::dec;
    }
    ostream << "]";

    if (remains < length) sig_named.emit(log("error", "packet", "malformed sub-payload detected. "  + ostream.str()));
    else                  sig_named.emit(log("debug", "packet", "unknown sub-payload detected. " + ostream.str()));
  }
}


/*****************************************************************************
 ** Implementation [Human Friendly Accessors]
 *****************************************************************************/

ecl::Angle<double> Kobuki::getHeading() const
{
  ecl::Angle<double> heading;
  // raw data angles are in hundredths of a degree, convert to radians.
  heading = (static_cast<double>(inertia.data.angle) / 100.0) * ecl::pi / 180.0;
  return ecl::wrap_angle(heading - heading_offset);
}

double Kobuki::getAngularVelocity() const
{
  // raw data angles are in hundredths of a degree, convert to radians.
  return (static_cast<double>(inertia.data.angle_rate) / 100.0) * ecl::pi / 180.0;
}

/*****************************************************************************
 ** Implementation [Raw Data Accessors]
 *****************************************************************************/

void Kobuki::resetOdometry()
{
  diff_drive.reset();

  // Issue #274: use current imu reading as zero heading to emulate reseting gyro
  heading_offset = (static_cast<double>(inertia.data.angle) / 100.0) * ecl::pi / 180.0;
}

void Kobuki::getWheelJointStates(double &wheel_left_angle, double &wheel_left_angle_rate, double &wheel_right_angle,
                                 double &wheel_right_angle_rate)
{
  diff_drive.getWheelJointStates(wheel_left_angle, wheel_left_angle_rate, wheel_right_angle, wheel_right_angle_rate);
}

/**
 * @brief Use the current sensor data (encoders and gyro) to calculate an update for the odometry.
 *
 * This fuses current sensor data with the last updated odometry state to produce the new
 * odometry state. This will be usually done in the slot callback to the stream_data signal.
 *
 * It is important that this is called every time a data packet is received from the robot.
 *
 * @param pose_update : return the pose updates in this variable.
 * @param pose_update_rates : return the pose update rates in this variable.
 */
void Kobuki::updateOdometry(ecl::Pose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates)
{
  diff_drive.update(core_sensors.data.time_stamp, core_sensors.data.left_encoder, core_sensors.data.right_encoder,
                      pose_update, pose_update_rates);
}

/*****************************************************************************
 ** Commands
 *****************************************************************************/

void Kobuki::setLed(const enum LedNumber &number, const enum LedColour &colour)
{
  sendCommand(Command::SetLedArray(number, colour, kobuki_command.data));
}

void Kobuki::setDigitalOutput(const DigitalOutput &digital_output) {
  sendCommand(Command::SetDigitalOutput(digital_output, kobuki_command.data));
}

void Kobuki::setExternalPower(const DigitalOutput &digital_output) {
  sendCommand(Command::SetExternalPower(digital_output, kobuki_command.data));
}

//void Kobuki::playSound(const enum Sounds &number)
//{
//  sendCommand(Command::PlaySound(number, kobuki_command.data));
//}

void Kobuki::playSoundSequence(const enum SoundSequences &number)
{
  sendCommand(Command::PlaySoundSequence(number, kobuki_command.data));
}

bool Kobuki::setControllerGain(const unsigned char &type, const unsigned int &p_gain,
                               const unsigned int &i_gain, const unsigned int &d_gain)
{
  if ((firmware.flashed_major_version() < 2) && (firmware.flashed_minor_version() < 2)) {
    sig_warn.emit("Robot firmware doesn't support this function, so you must upgrade it. " \
                  "Consult how-to on: http://kobuki.yujinrobot.com/documentation/howtos/upgrading-firmware");
    sig_warn.emit("Robot firmware version is " + VersionInfo::toString(firmware.data.version)
                + "; latest version is " + firmware.current_version());
    return false;
  }

  sendCommand(Command::SetControllerGain(type, p_gain, i_gain, d_gain));
  return true;
}

bool Kobuki::getControllerGain()
{
  if ((firmware.flashed_major_version() < 2) && (firmware.flashed_minor_version() < 2)) {
    sig_warn.emit("Robot firmware doesn't support this function, so you must upgrade it. " \
                  "Consult how-to on: http://kobuki.yujinrobot.com/documentation/howtos/upgrading-firmware");
    sig_warn.emit("Robot firmware version is " + VersionInfo::toString(firmware.data.version)
                + "; latest version is " + firmware.current_version());
    return false;
  }

  sendCommand(Command::GetControllerGain());
  return true;
}

void Kobuki::setBaseControl(const double &linear_velocity, const double &angular_velocity)
{
  diff_drive.setVelocityCommands(linear_velocity, angular_velocity);
}

void Kobuki::sendBaseControlCommand()
{
  std::vector<double> velocity_commands_received;
  if( acceleration_limiter.isEnabled() ) {
    velocity_commands_received=acceleration_limiter.limit(diff_drive.pointVelocity());
  } else {
    velocity_commands_received=diff_drive.pointVelocity();
  }
  diff_drive.velocityCommands(velocity_commands_received);
  std::vector<short> velocity_commands = diff_drive.velocityCommands();
  //std::cout << "speed: " << velocity_commands[0] << ", radius: " << velocity_commands[1] << std::endl;
  sendCommand(Command::SetVelocityControl(velocity_commands[0], velocity_commands[1]));

  //experimental; send raw control command and received command velocity
  velocity_commands_debug=velocity_commands;
  velocity_commands_debug.push_back((short)(velocity_commands_received[0]*1000.0));
  velocity_commands_debug.push_back((short)(velocity_commands_received[1]*1000.0));
  sig_raw_control_command.emit(velocity_commands_debug);
}

/**
 * @brief Send the prepared command to the serial port.
 *
 * Need to be a bit careful here, because we have no control over how the user
 * is calling this - they may be calling from different threads (this is so for
 * kobuki_node), so we mutex protect it here rather than relying on the user
 * to do so above.
 *
 * @param command : prepared command template (see Command's static member functions).
 */
void Kobuki::sendCommand(Command command)
{
  if( !is_alive || !is_connected ) {
    //need to do something
    sig_debug.emit("Device state is not ready yet.");
    if( !is_alive     ) sig_debug.emit(" - Device is not alive.");
    if( !is_connected ) sig_debug.emit(" - Device is not connected.");
    //std::cout << is_enabled << ", " << is_alive << ", " << is_connected << std::endl;
    return;
  }
  command_mutex.lock();
  kobuki_command.resetBuffer(command_buffer);

  if (!command.serialise(command_buffer))
  {
    sig_error.emit("command serialise failed.");
  }
  command_buffer[2] = command_buffer.size() - 3;
  unsigned char checksum = 0;
  for (unsigned int i = 2; i < command_buffer.size(); i++)
    checksum ^= (command_buffer[i]);

  command_buffer.push_back(checksum);
  //check_device();
  serial.write((const char*)&command_buffer[0], command_buffer.size());

  sig_raw_data_command.emit(command_buffer);
  command_mutex.unlock();
}

bool Kobuki::enable()
{
  is_enabled = true;
  return true;
}

bool Kobuki::disable()
{
  setBaseControl(0.0f, 0.0f);
  sendBaseControlCommand();
  is_enabled = false;
  return true;
}

/**
 * @brief Print a list of all relevant sigslot connections.
 *
 * This includes both the kobuki driver signals as well as externally
 * connected slots. Useful for when you need to check if any of your
 * connections are dangling (often happens when you typo
 * the name of the sigslots connection).
 */
void Kobuki::printSigSlotConnections() const {

  std::cout << "========== Void ==========" << std::endl;
  ecl::SigSlotsManager<>::printStatistics();
  std::cout << "========= String =========" << std::endl;
  ecl::SigSlotsManager<const std::string&>::printStatistics();
  std::cout << "====== Button Event ======" << std::endl;
  ecl::SigSlotsManager<const ButtonEvent&>::printStatistics();
  std::cout << "====== Bumper Event ======" << std::endl;
  ecl::SigSlotsManager<const BumperEvent&>::printStatistics();
  std::cout << "====== Cliff Event =======" << std::endl;
  ecl::SigSlotsManager<const CliffEvent&>::printStatistics();
  std::cout << "====== Wheel Event =======" << std::endl;
  ecl::SigSlotsManager<const WheelEvent&>::printStatistics();
  std::cout << "====== Power Event =======" << std::endl;
  ecl::SigSlotsManager<const PowerEvent&>::printStatistics();
  std::cout << "====== Input Event =======" << std::endl;
  ecl::SigSlotsManager<const InputEvent&>::printStatistics();
  std::cout << "====== Robot Event =======" << std::endl;
  ecl::SigSlotsManager<const RobotEvent&>::printStatistics();
  std::cout << "====== VersionInfo =======" << std::endl;
  ecl::SigSlotsManager<const VersionInfo&>::printStatistics();
  std::cout << "===== Command Buffer =====" << std::endl;
  ecl::SigSlotsManager<const Command::Buffer&>::printStatistics();
}

} // namespace kobuki
