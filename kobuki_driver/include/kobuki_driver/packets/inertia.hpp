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
 * @file /include/kobuki_driver/modules/inertia.hpp
 *
 * Module for handling of inertia packet payloads.
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_INERTIA_DATA_HPP__
#define KOBUKI_INERTIA_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class Inertia : public packet_handler::payloadBase
{
public:
  struct Data {
    int16_t angle;
    int16_t angle_rate;
    unsigned char acc[3];
  } data;

  virtual ~Inertia() {};

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: kobuki_inertia: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 7;
    buildBytes(Header::Inertia, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.angle, byteStream);
    buildBytes(data.angle_rate, byteStream);
    buildBytes(data.acc[0], byteStream);
    buildBytes(data.acc[1], byteStream);
    buildBytes(data.acc[2], byteStream);
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
    buildVariable(data.angle, byteStream);
    buildVariable(data.angle_rate, byteStream);
    buildVariable(data.acc[0], byteStream);
    buildVariable(data.acc[1], byteStream);
    buildVariable(data.acc[2], byteStream);

    return true;
  }
};

} // namespace kobuki

#endif /* KOBUKI_INERTIA_DATA_HPP__ */

