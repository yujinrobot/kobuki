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
 * @file /include/kobuki_driver/modules/gp_input.hpp
 *
 * Module for handling of gpio data commands.
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_GP_INPUT_HPP__
#define KOBUKI_GP_INPUT_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include <vector>
#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class GpInput : public packet_handler::payloadBase
{
public:
  struct Data {
    Data() : analog_input(4) {}
    uint16_t digital_input;
    /**
     * This currently returns 4 unsigned shorts containing analog values that
     * vary between 0 and 4095. These represent the values coming in on the
     * analog pins.
     */
    std::vector<uint16_t> analog_input;
  } data;

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: kobuki_inertia: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 2 + 2*data.analog_input.size();
    buildBytes(Header::GpInput, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.digital_input, byteStream);
    for (unsigned int i = 0; i < data.analog_input.size(); ++i)
    {
      buildBytes(data.analog_input[i], byteStream);
    }
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: kobuki_inertia: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id, length;
    buildVariable(header_id, byteStream);
    buildVariable(length, byteStream);
    buildVariable(data.digital_input, byteStream);

    //for (unsigned int i = 0; i < data.analog_input.size(); ++i)
    // It's actually sending 7 16bit variables.
    // 0-3 : the analog pin inputs
    // 4 : ???
    // 5-6 : 0
    for (unsigned int i = 0; i < 4; ++i)
    {
      buildVariable(data.analog_input[i], byteStream);
    }
    for (unsigned int i = 0; i < 3; ++i) {
      uint16_t dummy;
      buildVariable(dummy, byteStream);
    }
    return true;
  }

};

} // namespace kobuki

#endif /* KOBUKI_GP_INPUT_HPP__ */

