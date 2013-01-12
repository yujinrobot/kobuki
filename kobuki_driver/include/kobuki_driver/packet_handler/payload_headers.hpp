/**
 * @file include/kobuki_driver/packet_handler/payload_headers.hpp
 *
 * @brief Byte id's for the individual payload headers.
 *
 * Each part of a kobuki packet carries one or more payload chunks. Each chunk
 * is id'd by one of the values here.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_PAYLOAD_HEADERS_HPP_
#define KOBUKI_PAYLOAD_HEADERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

class Header {
public:
  enum PayloadType {
  // Streamed payloads
  CoreSensors = 1, DockInfraRed = 3, Inertia = 4, Cliff = 5, Current = 6,

  // Service Payloads
  Hardware = 10, Firmware = 11, ThreeAxisGyro = 13, Eeprom = 15, GpInput = 16,

  UniqueDeviceID = 19, Reserved = 20
  };
};

} // namespace kobuki

#endif /* KOBUKI_PAYLOAD_HEADERS_HPP_ */
