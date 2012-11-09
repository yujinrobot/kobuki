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
 * @file /kobuki/src/driver/kobuki.cpp
 *
 * @brief Implementation for the kobuki device driver.
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

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
    shutdown_requested(false), is_enabled(false), is_connected(false), is_alive(false)
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
  this->parameters=parameters;
  protocol_version = parameters.protocol_version;
  std::string sigslots_namespace = parameters.sigslots_namespace;
  event_manager.init(sigslots_namespace);

  //checking device
  if( access( parameters.device_port.c_str(), F_OK ) == -1 ) {
      ecl::Sleep waiting(5); //for 5sec.
      event_manager.update(is_connected, is_alive);
      while( access( parameters.device_port.c_str(), F_OK ) == -1 ) {
        sig_info.emit("device is not exist. still waiting...");
        waiting();
      }
  }
  //try {
  serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
  /*} catch (...){
    std::cout << "exception raised." << std::endl;
    throw; //kobuki_node will handle this
    //exit(-1); //forcely exit here
  }*/
  is_connected = true;
  is_alive = true;

  serial.block(4000); // blocks by default, but just to be clear!
  serial.clear();
  ecl::PushAndPop<unsigned char> stx(2, 0);
  ecl::PushAndPop<unsigned char> etx(1);
  stx.push_back(0xaa);
  stx.push_back(0x55);
  packet_finder.configure(sigslots_namespace, stx, etx, 1, 64, 1, true);

  sig_version_info.connect(sigslots_namespace + std::string("/version_info"));
  sig_stream_data.connect(sigslots_namespace + std::string("/stream_data"));
  sig_raw_data_command.connect(sigslots_namespace + std::string("/raw_data_command"));
  //sig_serial_timeout.connect(sigslots_namespace+std::string("/serial_timeout"));

  sig_debug.connect(sigslots_namespace + std::string("/ros_debug"));
  sig_info.connect(sigslots_namespace + std::string("/ros_info"));
  sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));
  sig_error.connect(sigslots_namespace + std::string("/ros_error"));

  diff_drive.init();
  gate_keeper.init(parameters.enable_gate_keeper);

  // in case the user changed these from the defaults
  Battery::capacity = parameters.battery_capacity;
  Battery::low = parameters.battery_low;
  Battery::dangerous = parameters.battery_dangerous;

  /******************************************
   ** Get Version Info Commands
   *******************************************/
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
      sig_error.emit("device is not exist.");
      is_connected = false;
      is_alive = false;
      event_manager.update(is_connected, is_alive);

      if( serial.open() )
      {
        sig_info.emit("device is still open, closing it, and retry to open.");
        serial.close();
      }
      //try_open();
      ecl::Sleep waiting(5); //for 5sec.
      while( access( parameters.device_port.c_str(), F_OK ) == -1 ) {
        sig_info.emit("device is not exist. still waiting...");
        waiting();
      }
      serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
      if( serial.open() ) {
        sig_info.emit("device is conencted.");
        is_connected = true;
        is_alive = true;
        event_manager.update(is_connected, is_alive);
        sendCommand(Command::GetVersionInfo());
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
      }
      event_manager.update(is_connected, is_alive);
      continue;
    }
    else
    {
      std::ostringstream ostream;
      ostream << "kobuki_node : serial_read(" << n << ")";
      sig_debug.emit(ostream.str());
      // might be useful to send this to a topic if there is subscribers
//        static unsigned char last_char(buf[0]);
//        for( int i(0); i<n; i++ )
//        {
//          printf("%02x ", buf[i] );
//          if( last_char == 0xaa && buf[i] == 0x55 ) printf("\n");
//          last_char = buf[i];
//        }
    }

    if (packet_finder.update(buf, n)) // this clears packet finder's buffer and transfers important bytes into it
    {
      packet_finder.getBuffer(data_buffer); // get a reference to packet finder's buffer.

#if 0
      if( verbose )
      {
        printf("Packet: ");
        for( unsigned int i=0; i<data_buffer.size(); i++ )
        {
          printf("%02x ", data_buffer[i] );
          if( i != 0 && ((i%5)==0) ) printf(" ");
        }
      }
#endif
      // deserialise; first three bytes are not data.
      data_buffer.pop_front();
      data_buffer.pop_front();
      data_buffer.pop_front();

      while (data_buffer.size() > 1/*size of etx*/)
      {
        // std::cout << "header_id: " << (unsigned int)data_buffer[0] << " | ";
        // std::cout << "remains: " << data_buffer.size() << " | ";
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
            // the rest are only included on request
          case Header::Hardware:
            hardware.deserialise(data_buffer);
            sig_version_info.emit(VersionInfo(firmware.data.version, hardware.data.version));
            break;
          case Header::Firmware:
            firmware.deserialise(data_buffer);
            sig_version_info.emit(VersionInfo(firmware.data.version, hardware.data.version));
            break;
          default:
            std::stringstream ostream;
            ostream << "unexpected sub-payload received [" << static_cast<unsigned int>(data_buffer[0]) << "]\n";
            ostream << "[";
            for (unsigned int i = 0; i < data_buffer.size(); ++i ) {
              ostream << std::hex << static_cast<int>(data_buffer[i]) << " " << std::dec;
            }
            ostream << "]";
            sig_error.emit(ostream.str());
            data_buffer.clear();
            break;
        }
      }

      is_alive = true;
      event_manager.update(is_connected, is_alive);
      last_signal_time.stamp();
      sig_stream_data.emit();
      sendBaseControlCommand(); // send the command packet to mainboard;
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
  diff_drive.velocityCommands(linear_velocity, angular_velocity);
}

void Kobuki::sendBaseControlCommand()
{
  std::vector<short> velocity_commands = diff_drive.velocityCommands();
  gate_keeper.confirm(velocity_commands[0], velocity_commands[1]);
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
    sig_debug.emit("device state is not ready yet.");
    if( !is_alive     ) sig_debug.emit(" - device is not alive.");
    if( !is_connected ) sig_debug.emit(" - device is not connected.");
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

} // namespace kobuki
