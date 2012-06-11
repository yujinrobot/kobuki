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
 * @file /kobuki_driver/include/kobuki_driver/modules/gate_keeper.hpp
 *
 * @brief Simple module for the diff drive odometry.
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_GATE_KEEPER_HPP_
#define KOBUKI_GATE_KEEPER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdint.h>
#include <ecl/time.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class GateKeeper {
public:
  GateKeeper() :
    is_enabled(true),
    last_speed(0), 
    last_timestamp(ecl::TimeStamp())
  {}
  void init(bool enable_gate_keeper){ is_enabled = enable_gate_keeper; }
  void confirm(short &speed, short &radius) // or smoother, limiter, etc
  {
    if( is_enabled )
    {
      //get current time
      ecl::TimeStamp curr_timestamp;
      //get time difference
      ecl::TimeStamp duration = curr_timestamp - last_timestamp;
      //calculate acceleration
      double acceleration = ((double)(speed - last_speed)) / duration; // in mm/s^2

      //if criterion meet some condition, limit input velocity in certain step
//      std::ostringstream oss;
//      oss << "[" << duration << "]";
//      oss << "[" << last_speed << "]";
//      oss << "[" << speed << ", " << radius << "]";
//      oss << "[" << acceleration << "]";

      if( std::abs(acceleration) > 20.0 ) // 20mm/s^2 ?
      { 
        if( std::abs(speed - last_speed) > 8 ) 
        {
          speed = last_speed + 8 * (short)(acceleration / std::abs(acceleration));
        }
      }
      if( radius == 0 && std::abs(speed) > 0 ) {
        radius = last_radius;
      } else { 
        last_radius = radius;
      }
 //     oss << "[" << speed << ", " << radius << "]";
 //     std::cout << oss.str() << std::endl;

      //update last_speed
      last_speed = speed;
      //update last_timestamp
      last_timestamp = curr_timestamp;
      
    }
  }

private:
  bool is_enabled;
  short last_speed;
  short last_radius;
//  unsigned short last_timestamp;
  ecl::TimeStamp last_timestamp;
};

} // namespace kobuki

#endif /* KOBUKI_GATE_KEEPER_HPP_ */
