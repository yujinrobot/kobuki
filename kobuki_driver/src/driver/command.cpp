/**
 * @file src/driver/command.cpp
 *
 * @brief Implementation of the command packets.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
**/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/command.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Static variables initialization
*****************************************************************************/

const unsigned char Command::header0 = 0xaa;
const unsigned char Command::header1 = 0x55;

/*****************************************************************************
** Implementation [Command Generators]
*****************************************************************************/

/**
 * Update the gp_out bits and get ready for sending as a SetDigitalOut
 * command.
 *
 * The led arrays are obtained from the gp_outputs with a 0x0f00 mask.
 *
 * - Led1 Red    : 0x0100
 * - Led1 Green  : 0x0200
 * - Led1 Orange : 0x0300
 * - Led2 Red    : 0x0400
 * - Led2 Green  : 0x0800
 * - Led2 Orange : 0x0c00
 *
 * Important to overlay this on top of the existing gp_out configuration.
 *
 * @param number : led enumerated number
 * @param colour : green, orange, red or black
 * @param current_data : need to store settings as the gp_output command is a combo command
 * @return Command : the command to send down the wire.
 */
Command Command::SetLedArray(const enum LedNumber &number, const enum LedColour &colour, Command::Data &current_data)
{
  // gp_out is 16 bits
  uint16_t value;
  if (number == Led1)
  {
    value = colour; // defined with the correct bit specification.
    current_data.gp_out = (current_data.gp_out & 0xfcff) | value; // update first
  }
  else
  {
    value = colour << 2;
    current_data.gp_out = (current_data.gp_out & 0xf3ff) | value; // update first
  }
  Command outgoing;
  outgoing.data = current_data;
  outgoing.data.command = Command::SetDigitalOut;
  return outgoing;
}

/**
 * Set one of the digital out pins available to the user.
 *
 * They set the last 4 bits on the data.gp_out variable.
 *
 * @todo could use far better documentation here/example here.
 *
 * @param digital_output : mask and value to send
 * @param current_data : need to store settings as the gp_output command is a combo command
 * @return Command : the command to send down the wire.
 */
Command Command::SetDigitalOutput(const DigitalOutput &digital_output, Command::Data &current_data)
{
  uint16_t values = 0x0000;
  uint16_t clear_mask = 0xfff0;
  for ( unsigned int i = 0; i < 4; ++i ) {
    if ( digital_output.mask[i] ) {
      if ( digital_output.values[i] ) {
        values |= ( 1 << i );
      }
    } else {
      clear_mask |= ( 1 << i ); // don't clear this bit, so set a 1 here
    }
  }
  current_data.gp_out = (current_data.gp_out & clear_mask) | values;
  Command outgoing;
  outgoing.data = current_data;
  outgoing.data.command = Command::SetDigitalOut;
  return outgoing;
}

/**
 * Set one of the external power sources available to the user.
 *
 * They set the second 4 bits(0x00f0) on the data.gp_out variable.
 *
 * @todo could use far better documentation here/example here.
 *
 * @param digital_output : mask and value to send
 * @param current_data : need to store settings as the gp_output command is a combo command
 * @return Command : the command to send down the wire.
 */
Command Command::SetExternalPower(const DigitalOutput &digital_output, Command::Data &current_data)
{
  uint16_t values = 0x0000;
  uint16_t clear_mask = 0xff0f;
  for ( unsigned int i = 0; i < 4; ++i ) {
    if ( digital_output.mask[i] ) {
      if ( digital_output.values[i] ) {
        values |= ( 1 << (i+4) );
      }
    } else {
      clear_mask |= ( 1 << (i+4) ); // don't clear this bit, so set a 1 here
    }
  }
  current_data.gp_out = (current_data.gp_out & clear_mask) | values;
  Command outgoing;
  outgoing.data = current_data;
  outgoing.data.command = Command::SetDigitalOut;
  return outgoing;
}

Command Command::PlaySoundSequence(const enum SoundSequences &number, Command::Data &current_data)
{
  uint16_t value; // gp_out is 16 bits
  value = number; // defined with the correct bit specification.

  Command outgoing;
  outgoing.data.segment_name = value;
  outgoing.data.command = Command::SoundSequence;
  return outgoing;
}

Command Command::GetVersionInfo()
{
  Command outgoing;
  outgoing.data.request_flags = 0;
  outgoing.data.request_flags |= static_cast<uint16_t>(HardwareVersion);
  outgoing.data.request_flags |= static_cast<uint16_t>(FirmwareVersion);
  outgoing.data.request_flags |= static_cast<uint16_t>(UniqueDeviceID);
  outgoing.data.command = Command::RequestExtra;
  return outgoing;
}

Command Command::SetVelocityControl(DiffDrive& diff_drive)
{
  Command outgoing;
  std::vector<short> velocity_commands = diff_drive.velocityCommands();
  outgoing.data.speed = velocity_commands[0];
  outgoing.data.radius = velocity_commands[1];
  outgoing.data.command = Command::BaseControl;
  return outgoing;
}
Command Command::SetVelocityControl(const int16_t &speed, const int16_t &radius)
{
  Command outgoing;
  outgoing.data.speed = speed;
  outgoing.data.radius = radius;
  outgoing.data.command = Command::BaseControl;
  return outgoing;
}

/*****************************************************************************
** Implementation [Serialisation]
*****************************************************************************/
/**
 * Clears the command buffer and resets the header.
 */
void Command::resetBuffer(Buffer& buffer) {
  buffer.clear();
  buffer.resize(64);
  buffer.push_back(Command::header0);
  buffer.push_back(Command::header1);
  buffer.push_back(0); // just initialise, we usually write in the payload here later (size of payload only, not stx, not etx, not length)
}

bool Command::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (!(byteStream.size() > 0))
  {
    //ROS_WARN_STREAM("kobuki_node: kobuki_command: serialise failed. empty byte stream.");
    return false;
  }
  // need to be sure we don't pass through an emum to the Trans'd buildBytes functions.
  unsigned char cmd = static_cast<unsigned char>(data.command);
  unsigned char length = 0;
  switch (data.command)
  {
    case BaseControl:
      buildBytes(cmd, byteStream);
      buildBytes(length=4, byteStream);
      buildBytes(data.speed, byteStream);
      buildBytes(data.radius, byteStream);
      break;
    case Sound:
      buildBytes(cmd, byteStream);
      buildBytes(length=3, byteStream);
      buildBytes(data.note, byteStream);
      buildBytes(data.duration, byteStream);
      break;
    case SoundSequence:
      buildBytes(cmd, byteStream);
      buildBytes(length=1, byteStream);
      buildBytes(data.segment_name, byteStream);
      break;
    case RequestExtra:
      buildBytes(cmd, byteStream);
      buildBytes(length=2, byteStream);
      buildBytes(data.request_flags, byteStream);
      break;
    case ChangeFrame:
      buildBytes(cmd, byteStream);
      buildBytes(length=1, byteStream);
      buildBytes(data.frame_id, byteStream);
      break;
    case RequestEeprom:
      buildBytes(cmd, byteStream);
      buildBytes(length=1, byteStream);
      buildBytes(data.frame_id, byteStream);
      break;
    case SetDigitalOut:
    { // this one controls led, external power sources, gp digitial output
      buildBytes(cmd, byteStream);
      buildBytes(length=2, byteStream);
      buildBytes(data.gp_out, byteStream);
      break;
    }
    default:
      return false;
      break;
  }
  return true;
}



} // namespace kobuki
