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
  static const unsigned char DockInfraRed = 3;
  static const unsigned char Inertia = 4;
  static const unsigned char Cliff = 5;
  static const unsigned char Current = 6;

  // Service Payloads
  static const unsigned char Hardware = 10;
  static const unsigned char Firmware = 11;
  static const unsigned char Eeprom = 15;
  static const unsigned char GpInput = 16;

  static const unsigned char Reserved = 20;
};

} // namespace kobuki

#endif /* KOBUKI_PAYLOAD_HEADERS_HPP_ */
