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
 * @file /include/kobuki_driver/modules/firmware.hpp
 *
 * Module for handling of firmware version request packet payloads.
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_FW_DATA_HPP__
#define KOBUKI_FW_DATA_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Constants
*****************************************************************************/

#define CURRENT_FIRMWARE_VERSION  10000   // i.e. 1.0.0; patch number is ignored

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class Firmware : public packet_handler::payloadBase
{
public:
  struct Data {
    uint16_t version;
  } data;

  // methods
  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_fw: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 2;
    buildBytes(Header::Firmware, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.version, byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      printf("kobuki_node: kobuki_fw: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id, length;
    buildVariable(header_id, byteStream);
    buildVariable(length, byteStream);
    buildVariable(data.version, byteStream);

    // TODO firmware currently hardcoded version is 123, but we want 10000 (1.0.0);
    // remove this horrible, dirty hack as soon as we upgrade the firmware to 101
    if (data.version == 123)
      data.version = 10000;

    //showMe();
    return constrain();
  }

  bool constrain()
  {
    return (data.version >= 10000);  // lowest version is 1.0.0
  }

  void showMe()
  {
    //printf("--[%02x || %03d | %03d | %03d]\n", data.bump, acc[2], acc[1], acc[0] );
  }

  std::string flashed_version() { return toString(data.version); }
  std::string current_version() { return toString(CURRENT_FIRMWARE_VERSION); }

  std::string toString(uint16_t version_number)
  {
    // Convert a short unsigned into a string of type <mayor>.<minor>.<patch>
    std::stringstream ss;
    ss << version_number;
    std::string version(ss.str());
    ss.str("");
    ss << version.substr(0, 1) << '.' << version.substr(1, 2) << '.' << version.substr(3, 2);
    version = ss.str();
    std::size_t pos = version.find(".0");
    while (pos != std::string::npos)
    {
      if (version[pos + 2] != '.')
        version.erase(pos + 1, 1);

      pos = version.find(".0", pos + 2);
    }

    return version;
  }

  int check_mayor_version()
  {
    // Return a negative value if firmware's mayor version is older than that of the driver,
    // 0 if both are the same, and a positive value if firmware's mayor version is newer
    std::stringstream ss; ss << data.version;
    std::string flashed_version(ss.str());
    ss.str(""); ss << CURRENT_FIRMWARE_VERSION;
    std::string current_version(ss.str());

    return flashed_version.compare(0, 1, current_version, 0, 1);
  }

  int check_minor_version()
  {
    // Return a negative value if firmware's minor version is older than that of the driver,
    // 0 if both are the same, and a positive value if firmware's minor version is newer
    std::stringstream ss; ss << data.version;
    std::string flashed_version(ss.str());
    ss.str(""); ss << CURRENT_FIRMWARE_VERSION;
    std::string current_version(ss.str());

    return flashed_version.compare(1, 2, current_version, 1, 2);
  }
};

} // namespace kobuki

#endif /* KOBUKI_FW_DATA_HPP__ */

