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

#include <vector>
#include <iomanip>
#include <sstream>
#include <iostream>
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
  void init(bool enable_acceleration_limiter
    , double linear_acceleration_max_= 0.5, double angular_acceleration_max_= 3.5
    , double linear_deceleration_max_=-0.5*1.2, double angular_deceleration_max_=-3.5*1.2)
  {
    is_enabled = enable_acceleration_limiter;
    linear_acceleration_max  = linear_acceleration_max_ ;
    linear_deceleration_max  = linear_deceleration_max_ ;
    angular_acceleration_max = angular_acceleration_max_;
    angular_deceleration_max = angular_deceleration_max_;
  }

  bool isEnabled() const { return is_enabled; }

  /**
   * @brief Limits the input velocity commands if gatekeeper is enabled.
   *
   * What is the limit?
   *
   * @param speed : ..
   * @param radius : ..
   */
  std::vector<double> limit(const std::vector<double> &command) { return limit(command[0], command[1]); }

  std::vector<double> limit(const double &vx, const double &wz)
  {
    if( is_enabled ) {
      //get current time
      ecl::TimeStamp curr_timestamp;
      //get time difference
      ecl::TimeStamp duration = curr_timestamp - last_timestamp;
      //calculate acceleration
      double linear_acceleration = ((double)(vx - last_vx)) / duration; // in [m/s^2]
      double angular_acceleration = ((double)(wz - last_wz)) / duration; // in [rad/s^2]

      //std::ostringstream oss;
      //oss << std::fixed << std::setprecision(4);
      //oss << "[" << std::setw(6) << (double)duration << "]";
      //oss << "[" << std::setw(6) << last_vx << ", " << std::setw(6) << last_wz << "]";
      //oss << "[" << std::setw(6) << vx << ", " << std::setw(6) << wz << "]";
      //oss << "[" << std::setw(6) << linear_acceleration << ", " << std::setw(6) << angular_acceleration << "]";

      if( linear_acceleration > linear_acceleration_max )
        command_vx = last_vx + linear_acceleration_max * duration;
      else if( linear_acceleration < linear_deceleration_max )
        command_vx = last_vx + linear_deceleration_max * duration;
      else
        command_vx = vx;
      last_vx = command_vx;

      if( angular_acceleration > angular_acceleration_max )
        command_wz = last_wz + angular_acceleration_max * duration;
      else if( angular_acceleration < angular_deceleration_max )
        command_wz = last_wz + angular_deceleration_max * duration;
      else
        command_wz = wz;
      last_wz = command_wz;

      last_timestamp = curr_timestamp;

      //oss << "[" << std::setw(6) << command_vx << ", " << std::setw(6) << command_wz << "]";
      //std::cout << oss.str() << std::endl;

      std::vector<double> ret_val;
      ret_val.push_back(command_vx);
      ret_val.push_back(command_wz);
      return ret_val;
    }
  }

private:
  bool is_enabled;
  short last_speed;
  short last_radius;
//  unsigned short last_timestamp;
  ecl::TimeStamp last_timestamp;

  double last_vx, last_wz; // In [m/s] and [rad/s]
  double command_vx, command_wz; // In [m/s] and [rad/s]
  double linear_acceleration_max, linear_deceleration_max; // In [m/s^2]
  double angular_acceleration_max, angular_deceleration_max; // In [rad/s^2]
};

} // namespace kobuki

#endif /* KOBUKI_ACCELERATION_LIMITER__HPP_ */
