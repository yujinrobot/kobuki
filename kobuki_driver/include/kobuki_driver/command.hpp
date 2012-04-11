#ifndef KOBUKI_COMMAND_DATA_HPP__
#define KOBUKI_COMMAND_DATA_HPP__

#include <ecl/containers.hpp>
#include "packet_handler/payload_base.hpp"
#include <kobuki_comms/Header.h>
#include "led_array.hpp"

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

  struct Data {
    Data() :
      command(BaseControl),
      speed(0),
      radius(0),
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
  void update(const enum LedNumber &number, const enum LedColour &colour) {
    // gp_out is 16 bits
    uint16_t value;
    if ( number == Led1 ) {
      value = colour;  // defined with the correct bit specification.
      data.gp_out = ( data.gp_out & 0xfcff ) | value; // update first
    } else {
      value = colour << 2;
      data.gp_out = ( data.gp_out & 0xf3ff ) | value; // update first
    }
  }

  Data data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_command: serialise failed. empty byte stream.");
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

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      ROS_WARN_STREAM("kobuki_node: kobuki_command: deserialise failed. empty byte stream.");
      return false;
    }
    return true;

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, acc[2], acc[1], acc[0] );
  }
};

} // namespace kobuki

#endif /* KOBUKI_COMMAND_DATA_HPP__ */

