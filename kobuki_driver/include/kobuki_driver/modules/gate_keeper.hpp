/**
 * @file /kobuki_driver/include/kobuki_driver/modules/gate_keeper.hpp
 * @author Younghun Ju <younghoon.ju@rnd.yujinrobot.com> <yhju83@gmail.com>
 * @brief Simple module for the velocity smoothing.
 * @date March 2012
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
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
