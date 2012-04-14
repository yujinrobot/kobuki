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
#ifndef KOBUKI_COMMAND_DATA_HPP__
#define KOBUKI_COMMAND_DATA_HPP__

#include <ecl/containers.hpp>
#include "packet_handler/payload_base.hpp"
#include "modules/led_array.hpp"

namespace kobuki
{

class Command : public packet_handler::payloadBase
{
public:
  // These values are very important - they go into the packet command id byte
  enum Name {
    BaseControl = 1,
    Sound = 3,
    SoundSequence = 4,
    RequestExtra = 9,
    ChangeFrame = 10,
    RequestEeprom = 11,
    SetDigitalOut = 12
  };

  enum RequestExtraFlag {
    HardwareVersion = 0x01,
    FirmwareVersion = 0x02
    // Time = 0x04
  };

  /**
   * Data structure containing data for commands. It is important to keep this
   * state as it will have to retain knowledge of the last known command in
   * some instances - e.g. for gp_out commands, quite often the incoming command
   * is only to set the output for a single led while keeping the rest of the current
   * gp_out values as is.
   *
   * For generating individual commands we modify the data here, then copy the command
   * class (avoid doing mutexes) and spin it off for sending down to the device.
   */
  struct Data {
    Data() :
      command(BaseControl),
      speed(0),
      radius(0),
      request_flags(0),
      gp_out(0x00f0) // set all the power pins high, others low.
    {}

    Name command;

    // BaseControl
    uint16_t speed;
    uint16_t radius;

    // Sound
    uint16_t note;
    unsigned char duration;

    // SoundSequence
    unsigned char segment_name;

    // RequestExtra
    uint16_t request_flags;

    // ChangeFrame & RequestEeprom
    unsigned char frame_id;

    // SetDigitalOut
    uint16_t gp_out;
  };

  virtual ~Command() {}

  /******************************************
  ** Command Wizards
  *******************************************/
  /**
   * Update the gp_out bits and get ready for sending as a SetDigitalOut
   * command.
   *
   * Important to overlay this on top of the existing gp_out configuration.
   *
   * @param number : led enumerated number
   * @param colour : green, orange, red or black
   */
  static Command SetLedArray(const enum LedNumber &number, const enum LedColour &colour, Command::Data &current_data) {
    // gp_out is 16 bits
    uint16_t value;
    if ( number == Led1 ) {
      value = colour;  // defined with the correct bit specification.
      current_data.gp_out = ( current_data.gp_out & 0xfcff ) | value; // update first
    } else {
      value = colour << 2;
      current_data.gp_out = ( current_data.gp_out & 0xf3ff ) | value; // update first
    }
    Command outgoing;
    outgoing.data = current_data;
    outgoing.data.command = Command::SetDigitalOut;
    return outgoing;
  }

  static Command GetVersionInfo() {
    Command outgoing;
    outgoing.data.request_flags = 0;
    outgoing.data.request_flags |= static_cast<uint16_t>(HardwareVersion);
    outgoing.data.request_flags |= static_cast<uint16_t>(FirmwareVersion);
    outgoing.data.command = Command::RequestExtra;
    return outgoing;
  }

  Data data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: kobuki_command: serialise failed. empty byte stream.");
      return false;
    }
    // need to be sure we don't pass through an emum to the Trans'd buildBytes functions.
    unsigned char cmd = static_cast<unsigned char>(data.command);
    switch (data.command)
    {
      case BaseControl:
        buildBytes(cmd, byteStream);
        buildBytes(data.speed, byteStream);
        buildBytes(data.radius, byteStream);
        break;
      case Sound:
        buildBytes(cmd, byteStream);
        buildBytes(data.note, byteStream);
        buildBytes(data.duration, byteStream);
        break;
      case SoundSequence:
        buildBytes(cmd, byteStream);
        buildBytes(data.segment_name, byteStream);
        break;
      case RequestExtra:
        buildBytes(cmd, byteStream);
        buildBytes(data.request_flags, byteStream);
        break;
      case ChangeFrame:
        buildBytes(cmd, byteStream);
        buildBytes(data.frame_id, byteStream);
        break;
      case RequestEeprom:
        buildBytes(cmd, byteStream);
        buildBytes(data.frame_id, byteStream);
        break;
      case SetDigitalOut: { // this one controls led, external power sources, gp digitial output
        buildBytes(cmd, byteStream);
        buildBytes(data.gp_out, byteStream);
        break;
      }
      default:
        return false;
        break;
    }
    return true;
  }

  /**
   * Unused.
   */
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream) { return true; }
};

} // namespace kobuki

#endif /* KOBUKI_COMMAND_DATA_HPP__ */

