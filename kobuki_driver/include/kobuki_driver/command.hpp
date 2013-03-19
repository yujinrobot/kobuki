/**
 * @file include/kobuki_driver/command.hpp
 *
 * @brief Command structure.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_COMMAND_DATA_HPP__
#define KOBUKI_COMMAND_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/containers.hpp>
#include "packet_handler/payload_base.hpp"
#include "modules/led_array.hpp"
#include "modules.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/


namespace kobuki
{

class Command : public packet_handler::payloadBase
{
public:
  typedef ecl::PushAndPop<unsigned char> Buffer;
  typedef ecl::Stencil< Buffer > BufferStencil;

  /**
   * These values are used to detect the type of sub-payload that is ensuing.
   */
  enum Name
  {
    BaseControl = 1, Sound = 3, SoundSequence = 4, RequestExtra = 9, ChangeFrame = 10, RequestEeprom = 11,
    SetDigitalOut = 12
  };

  enum VersionFlag
  {
    HardwareVersion = 0x01, FirmwareVersion = 0x02/*, Time = 0x04*/, UniqueDeviceID = 0x08
  };

  /**
   * @brief Data structure containing data for commands.
   *
   * It is important to keep this
   * state as it will have to retain knowledge of the last known command in
   * some instances - e.g. for gp_out commands, quite often the incoming command
   * is only to set the output for a single led while keeping the rest of the current
   * gp_out values as is.
   *
   * For generating individual commands we modify the data here, then copy the command
   * class (avoid doing mutexes) and spin it off for sending down to the device.
   */
  struct Data
  {
    Data() :
        command(BaseControl), speed(0), radius(0), request_flags(0), gp_out(0x00f0) // set all the power pins high, others low.
    {
    }

    Name command;

    // BaseControl
    int16_t speed;
    int16_t radius;

    // Sound - not yet implemented
    uint16_t note;
    unsigned char duration;

    // SoundSequence
    // 0 - turning on, 1 - turn off, 2 - recharge start, 3 - press button,
    // 4 - error sound, 5 - start cleaning, 6 - cleaning end
    unsigned char segment_name;

    // RequestExtra (version flags)
    uint16_t request_flags;

    // ChangeFrame & RequestEeprom
    unsigned char frame_id;

    // SetDigitalOut
    // 0x000f - digital output pins 0-3 (0x0001, 0x0002, 0x0004, 0x0008)
    // 0x00f0 - external power breakers (3.3V, 5V, 12V 12V1A) (0x0010, 0x0020, 0x0040, 0x0080)
    // 0x0f00 - led array (red1, green1, red2, green2) ( 0x0100, 0x0200, 0x0400, 0x0800)
    uint16_t gp_out;
  };

  virtual ~Command() {}

  static Command SetLedArray(const enum LedNumber &number, const enum LedColour &colour, Command::Data &current_data);
  static Command SetDigitalOutput(const DigitalOutput &digital_output, Command::Data &current_data);
  static Command SetExternalPower(const DigitalOutput &digital_output, Command::Data &current_data);
  static Command PlaySoundSequence(const enum SoundSequences &number, Command::Data &current_data);
  static Command GetVersionInfo();
  static Command SetVelocityControl(DiffDrive& diff_drive);
  static Command SetVelocityControl(const int16_t &speed, const int16_t &radius);

  Data data;

  void resetBuffer(Buffer &buffer);
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream) { return true; } /**< Unused **/

private:
  static const unsigned char header0;
  static const unsigned char header1;

};

} // namespace kobuki

#endif /* KOBUKI_COMMAND_DATA_HPP__ */

