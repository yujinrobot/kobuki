/**
 * @file /kobuki_driver/include/kobuki_driver/modules/acceleration_limiter.hpp
 *
 * @brief Simple module for the velocity smoothing.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_ACCELERATION_LIMITER_HPP_
#define KOBUKI_ACCELERATION_LIMITER_HPP_

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

/**
 * @brief An acceleration limiter for the kobuki.
 *
 * This class will check incoming velocity commands and limit them if
 * the change since the last incoming command is great.
 *
 * Right now, this hasn't got any configurable parameters for the user -
 * that might be an option to provide for users in the future. Ideally
 *
 * - User can disable this and do their own velocity smoothing outside.
 * - User can enable this with defaults (fairly high accelerations)
 * - (Later) User can enable this reconfigure a parameter to suit.
 */
class AccelerationLimiter {
public:
  AccelerationLimiter() :
    is_enabled(true),
    last_speed(0), 
    last_timestamp(ecl::TimeStamp())
  {}
  void init(bool enable_acceleration_limiter){ is_enabled = enable_acceleration_limiter; }
  /**
   * @brief Limits the input velocity commands if gatekeeper is enabled.
   *
   * What is the limit?
   *
   * @param speed : ..
   * @param radius : ..
   */
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
        if( std::abs(speed - last_speed) > 8 ) // this might have been experimentally determined, possibly becoming incorrect if the kobuki update rate changes.
        {
          speed = last_speed + 8 * (short)(acceleration / std::abs(acceleration));
        }
      }
      /*
      if( radius == 0 && std::abs(speed) > 0 ) {
        radius = last_radius;
      } else { 
        last_radius = radius;
      }
      */
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

#endif /* KOBUKI_ACCELERATION_LIMITER__HPP_ */
