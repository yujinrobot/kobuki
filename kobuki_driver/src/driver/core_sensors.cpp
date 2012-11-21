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
 * @file /kobuki_driver/src/driver/core_sensors.cpp
 *
 * @brief Implementation of the core sensor packet data.
 *
 * @date 06/04/2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/packets/core_sensors.hpp"
#include "../../include/kobuki_driver/packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool CoreSensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (!(byteStream.size() > 0))
  {
    //ROS_WARN_STREAM("kobuki_node: kobuki_default: serialise failed. empty byte stream.");
    return false;
  }
  unsigned char length = 15;
  buildBytes(Header::CoreSensors, byteStream);
  buildBytes(length, byteStream);
  buildBytes(data.time_stamp, byteStream);	//2
  buildBytes(data.bumper, byteStream);		//1
  buildBytes(data.wheel_drop, byteStream);	//1
  buildBytes(data.cliff, byteStream);		//1
  buildBytes(data.left_encoder, byteStream);	//2
  buildBytes(data.right_encoder, byteStream);	//2
  buildBytes(data.left_pwm, byteStream);	//1
  buildBytes(data.right_pwm, byteStream);	//1
  buildBytes(data.buttons, byteStream);		//1
  buildBytes(data.charger, byteStream);		//1
  buildBytes(data.battery, byteStream);		//1
  buildBytes(data.over_current, byteStream);	//1

  return true;
}
bool CoreSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (!(byteStream.size() > 0))
  {
    //ROS_WARN_STREAM("kobuki_node: kobuki_default: deserialise failed. empty byte stream.");
    return false;
  }

  unsigned char header_id, length;
  buildVariable(header_id, byteStream);
  buildVariable(length, byteStream);
  buildVariable(data.time_stamp, byteStream);
  buildVariable(data.bumper, byteStream);
  buildVariable(data.wheel_drop, byteStream);
  buildVariable(data.cliff, byteStream);
  buildVariable(data.left_encoder, byteStream);
  buildVariable(data.right_encoder, byteStream);
  buildVariable(data.left_pwm, byteStream);
  buildVariable(data.right_pwm, byteStream);
  buildVariable(data.buttons, byteStream);
  buildVariable(data.charger, byteStream);
  buildVariable(data.battery, byteStream);
  buildVariable(data.over_current, byteStream);

  return true;
}



} // namespace kobuki
