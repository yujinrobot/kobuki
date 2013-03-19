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
  sig_stream_data.connect(sigslots_namespace + std::string("/stream_data"));
  sig_raw_data_command.connect(sigslots_namespace + std::string("/raw_data_command"));
  sig_raw_data_stream.connect(sigslots_namespace + std::string("/raw_data_stream"));
  //sig_serial_timeout.connect(sigslots_namespace+std::string("/serial_timeout"));

  sig_debug.connect(sigslots_namespace + std::string("/ros_debug"));
  sig_info.connect(sigslots_namespace + std::string("/ros_info"));
  sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));
  sig_error.connect(sigslots_namespace + std::string("/ros_error"));

  //checking device
  if (access(parameters.device_port.c_str(), F_OK) == -1)
  {
    ecl::Sleep waiting(5); //for 5sec.
    event_manager.update(is_connected, is_alive);
    while (access(parameters.device_port.c_str(), F_OK) == -1)
    {
      sig_info.emit("Device does not exist. Waiting...");
      waiting();
    }
  }

  serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);

  is_connected = true;
  is_alive = true;

  serial.block(4000); // blocks by default, but just to be clear!
  serial.clear();
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

  thread.start(&Kobuki::spin, *this);
}

/*****************************************************************************
 ** Implementation [Runtime]
 *****************************************************************************/

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
    if( access( parameters.device_port.c_str(), F_OK ) == -1 ) {
      sig_error.emit("Device does not exist.");
      is_connected = false;
      is_alive = false;
      event_manager.update(is_connected, is_alive);

      if( serial.open() )
      {
        sig_info.emit("Device is still open, closing it and will try to open it again.");
        serial.close();
      }
      //try_open();
      ecl::Sleep waiting(5); //for 5sec.
      while( access( parameters.device_port.c_str(), F_OK ) == -1 ) {
        sig_info.emit("Device does not exist. Still waiting...");
        waiting();
      }
      serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
      if( serial.open() ) {
        sig_info.emit("device is connected.");
        is_connected = true;
        event_manager.update(is_connected, is_alive);
        version_info_reminder = 10;
      }
    }

    /*********************
     ** Read Incoming
     **********************/
    int n = serial.read(buf, packet_finder.numberOfDataToRead());
    if (n == 0)
    {
      if (is_alive && ((ecl::TimeStamp() - last_signal_time) > timeout))
      {
        is_alive = false;
        version_info_reminder = 10;
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
      sig_debug.emit(ostream.str());
      // might be useful to send this to a topic if there is subscribers
    }

    if (packet_finder.update(buf, n)) // this clears packet finder's buffer and transfers important bytes into it
    {
      packet_finder.getBuffer(data_buffer); // get a reference to packet finder's buffer.
      PacketFinder::BufferType local_buffer;
      local_buffer = data_buffer; //copy it to local_buffer, debugging purpose.
      sig_raw_data_stream.emit(local_buffer);

      // deserialise; first three bytes are not data.
      data_buffer.pop_front();
      data_buffer.pop_front();
      data_buffer.pop_front();

      while (data_buffer.size() > 1/*size of etx*/)
      {
        //std::cout << "header_id: " << (unsigned int)data_buffer[0] << " | ";
        //std::cout << "remains: " << data_buffer.size() << " | ";
        //std::cout << "local_buffer: " << local_buffer.size() << " | ";
        //std::cout << std::endl;
        switch (data_buffer[0])
        {
          // these come with the streamed feedback
          case Header::CoreSensors:
            core_sensors.deserialise(data_buffer);
            event_manager.update(core_sensors.data, cliff.data.bottom);
            break;
          case Header::DockInfraRed:
            dock_ir.deserialise(data_buffer);
            break;
          case Header::Inertia:
            inertia.deserialise(data_buffer);
            break;
          case Header::Cliff:
            cliff.deserialise(data_buffer);
            break;
          case Header::Current:
            current.deserialise(data_buffer);
            break;
          case Header::GpInput:
            gp_input.deserialise(data_buffer);
            event_manager.update(gp_input.data.digital_input);
            break;
          case Header::ThreeAxisGyro:
            three_axis_gyro.deserialise(data_buffer);
            break;
          // the rest are only included on request
          case Header::Hardware:
            hardware.deserialise(data_buffer);
            //sig_version_info.emit(VersionInfo(firmware.data.version, hardware.data.version));
            break;
          case Header::Firmware:
            firmware.deserialise(data_buffer);
            try
            {
              // Check firmware/driver compatibility; mayor version must be the same
              int version_match = firmware.check_mayor_version();
              if (version_match < 0) {
                sig_error.emit("Robot firmware is outdated and needs to be upgraded. Consult how-to on: " \
                               "http://kobuki.yujinrobot.com/documentation/howtos/upgrading-firmware");
                sig_warn.emit("version info - " + VersionInfo::toString(firmware.data.version)
                        + "; current version is " + firmware.current_version());
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
                  sig_warn.emit("version info - " + VersionInfo::toString(firmware.data.version)
                          + "; current version is " + firmware.current_version());
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
            unique_device_id.deserialise(data_buffer);
            sig_version_info.emit( VersionInfo( firmware.data.version, hardware.data.version
                , unique_device_id.data.udid0, unique_device_id.data.udid1, unique_device_id.data.udid2 ));
            sig_info.emit("version info - Hardware: " + VersionInfo::toString(hardware.data.version)
                                     + ". Firmware: " + VersionInfo::toString(firmware.data.version));
            version_info_reminder = 0;
            break;
          default:
            if (data_buffer.size() < 3 ) { /* minimum is 3, header_id, length, etx */
              sig_error.emit("malformed subpayload detected.");
              data_buffer.clear();
            } else {
              std::stringstream ostream;
              unsigned int header_id = static_cast<unsigned int>(data_buffer.pop_front());
              unsigned int length = static_cast<unsigned int>(data_buffer.pop_front());
              unsigned int remains = data_buffer.size();
              unsigned int to_pop;

              ostream << "[" << header_id << "]";
              ostream << "[" << length << "] ";

              ostream << "[";
              ostream << std::setfill('0') << std::uppercase;
              ostream << std::hex << std::setw(2) << header_id << " " << std::dec;
              ostream << std::hex << std::setw(2) << length << " " << std::dec;

              if (remains < length) to_pop = remains;
              else                  to_pop = length;

              for (unsigned int i = 0; i < to_pop; i++ ) {
                unsigned int byte = static_cast<unsigned int>(data_buffer.pop_front());
                ostream << std::hex << std::setw(2) << byte << " " << std::dec;
              }
              ostream << "]";

              if (remains < length) sig_error.emit("malformed sub-payload detected. "  + ostream.str());
              else                  sig_debug.emit("unexpected sub-payload received. " + ostream.str());

            }
            break;
        }
      }

      is_alive = true;
      event_manager.update(is_connected, is_alive);
      last_signal_time.stamp();
      sig_stream_data.emit();
      sendBaseControlCommand(); // send the command packet to mainboard;
      if( version_info_reminder/*--*/ > 0 ) sendCommand(Command::GetVersionInfo());
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

/*****************************************************************************
 ** Implementation [Human Friendly Accessors]
 *****************************************************************************/

ecl::Angle<double> Kobuki::getHeading() const
{
  ecl::Angle<double> heading;
  // raw data angles are in hundredths of a degree, convert to radians.
  heading = (static_cast<double>(inertia.data.angle) / 100.0) * ecl::pi / 180.0;
  return heading;
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
  diff_drive.reset(inertia.data.angle);
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

void Kobuki::setBaseControl(const double &linear_velocity, const double &angular_velocity)
{
  diff_drive.setVelocityCommands(linear_velocity, angular_velocity);
}

void Kobuki::sendBaseControlCommand()
{
  if( acceleration_limiter.isEnabled() ) {
    diff_drive.velocityCommands(acceleration_limiter.limit(diff_drive.pointVelocity()));
  } else {
    diff_drive.velocityCommands(diff_drive.pointVelocity());
  }
  std::vector<short> velocity_commands = diff_drive.velocityCommands();
  //std::cout << "speed: " << velocity_commands[0] << ", radius: " << velocity_commands[1] << std::endl;
  sendCommand(Command::SetVelocityControl(velocity_commands[0], velocity_commands[1]));
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
  serial.write(&command_buffer[0], command_buffer.size());

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
