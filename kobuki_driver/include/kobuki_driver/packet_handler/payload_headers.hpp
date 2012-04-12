/**
 * @file /kobuki_driver/include/kobuki_driver/packet_handler/payload_headers.hpp
 *
 * @brief Byte id's for the individual payload headers.
 *
 * Each part of a kobuki packet carries one or more payload chunks. Each chunk
 * is id'd by one of the values here.
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
  // Streamed payloads
  static const unsigned char CoreSensors = 1;
  static const unsigned char InfraRed = 2;
  static const unsigned char DockInfraRed = 3;
  static const unsigned char Inertia = 4;
  static const unsigned char Cliff = 5;
  static const unsigned char Current = 6;

  // Service Payloads
  static const unsigned char Hardware = 10;
  static const unsigned char Firmware = 11;
  static const unsigned char Eeprom = 15;
  static const unsigned char Gpio = 16;

  static const unsigned char Reserved = 20;
};

} // namespace kobuki

#endif /* KOBUKI_PAYLOAD_HEADERS_HPP_ */
