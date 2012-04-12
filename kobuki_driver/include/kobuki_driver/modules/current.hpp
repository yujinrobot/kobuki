/**
 * @file /include/kobuki_driver/modules/current.hpp
 *
 * Module for handling of current level packet payloads.
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_CURRENT_DATA_HPP__
#define KOBUKI_CURRENT_DATA_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include <vector>
#include "../packet_handler/payload_base.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/
/**
 * This comes back in the streamed feedback. It has two values
 * (left and right) indicating the supplied current which can be useful for
 * detecting when the robot is blocked.
 */
class Current : public packet_handler::payloadBase
{
public:
  struct Data {
    Data() : current(2) {}
    uint8_t header_id;
    std::vector<uint8_t> current;
  } data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_current: serialise failed. empty byte stream.");
      return false;
    }

    buildBytes(data.header_id, byteStream);
    buildBytes(data.current[0], byteStream);
    buildBytes(data.current[1], byteStream);
    //buildBytes( data.current[2], byteStream );
    //buildBytes( data.current[3], byteStream );
    //buildBytes( data.current[4], byteStream );
    //buildBytes( data.current[5], byteStream );
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_current: deserialise failed. empty byte stream.");
      return false;
    }

    buildVariable(data.header_id, byteStream);
    buildVariable(data.current[0], byteStream);
    buildVariable(data.current[1], byteStream);
    //buildVariable( data.current[2], byteStream );
    //buildVariable( data.current[3], byteStream );
    //buildVariable( data.current[4], byteStream );
    //buildVariable( data.current[5], byteStream );

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return true;
  }

  void showMe()
  {
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, current[2], current[1], current[0] );
  }
};

} // namespace kobuki

#endif /* KOBUKI_CURRENT_DATA_HPP__ */
