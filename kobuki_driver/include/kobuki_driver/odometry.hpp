/**
 * @file /kobuki_driver/include/kobuki_driver/odometry.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 06/04/2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_ODOMETRY_HPP_
#define KOBUKI_ODOMETRY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class Odometry {
public:
  // has the /odom information.
  struct Data {
  };
  struct JointStates {
    float wheel_left_radians, wheel_right_radians;
    float wheel_left_velocity, wheel_right_velocity;
  };

  Data data;
};

} // namespace kobuki

#endif /* KOBUKI_ODOMETRY_HPP_ */
