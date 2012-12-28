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
 * @file /include/kobuki_driver/modules/three_axis_gyro.hpp
 *
 * Module for handling of three_axis_gyro packet payloads.
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef KOBUKI_THREE_AXIS_GYRO_DATA_HPP__
#define KOBUKI_THREE_AXIS_GYRO_DATA_HPP__

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../packet_handler/payload_headers.hpp"

/*****************************************************************************
** Defines
*****************************************************************************/

#define MAX_DATA_SIZE (3*8) //derived from ST_GYRO_MAX_DATA_SIZE in firmware

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Interface
*****************************************************************************/

class ThreeAxisGyro : public packet_handler::payloadBase
{
public:
  struct Data {
    unsigned char frame_id;
    unsigned char followed_data_length;
    unsigned short data[MAX_DATA_SIZE];
  } data;

  virtual ~ThreeAxisGyro() {};

  bool serialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: three_axis_gyro: serialise failed. empty byte stream.");
      return false;
    }

    unsigned char length = 2 + 2 * data.followed_data_length;
    buildBytes(Header::ThreeAxisGyro, byteStream);
    buildBytes(length, byteStream);
    buildBytes(data.frame_id, byteStream);
    buildBytes(data.followed_data_length, byteStream);
    for (unsigned int i=0; i<data.followed_data_length; i++)
      buildBytes(data.data[i], byteStream);
    return true;
  }

  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream)
  {
    if (!(byteStream.size() > 0))
    {
      //ROS_WARN_STREAM("kobuki_node: three_axis_gyro: deserialise failed. empty byte stream.");
      return false;
    }

    unsigned char header_id, length;
    buildVariable(header_id, byteStream);
    buildVariable(length, byteStream);
    buildVariable(data.frame_id, byteStream);
    buildVariable(data.followed_data_length, byteStream);
    for (unsigned int i=0; i<data.followed_data_length; i++)
      buildVariable(data.data[i], byteStream);

    return true;
  }
};

} // namespace kobuki

#endif /* KOBUKI_THREE_AXIS_GYRO_DATA_HPP__ */

