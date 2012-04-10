/**
 * @file /kobuki_driver/include/kobuki_driver/simulation.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 10/04/2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SIMULATION_HPP_
#define SIMULATION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/time/timestamp.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/



class SimulationData {
public:
  SimulationData() :
    heading(0.0),
    velocity(0.0),
    angular_velocity(0.0),
    left_wheel_angle(0.0),
    right_wheel_angle(0.0),
    left_wheel_angle_update(0.0),
    right_wheel_angle_update(0.0),
    left_wheel_angle_rate(0.0),
    right_wheel_angle_rate(0.0),
    snooze(ecl::Duration(0.02))
    {
    }

  void init(const double &b, const double &metres_to_radians) {
    bias = b;
    m_to_rad = metres_to_radians;
  }

  void reset() {
    left_wheel_angle = 0.0;
    right_wheel_angle = 0.0;
    heading = 0.0;
  }

  void update() {
    ecl::TimeStamp stamp; // current stamp
    ecl::Duration diff = stamp - last_timestamp;
    left_wheel_angle_rate = (velocity - bias*angular_velocity/2.0) * m_to_rad;
    right_wheel_angle_rate = (velocity + bias*angular_velocity/2.0) * m_to_rad;
    left_wheel_angle_update = left_wheel_angle_rate * diff;
    right_wheel_angle_update = right_wheel_angle_rate * diff;
    left_wheel_angle += left_wheel_angle_update;
    right_wheel_angle += right_wheel_angle_update;

    heading += angular_velocity * diff;

    last_timestamp = stamp;
//    std::cout << "[" << last_timestamp << "][" << left_wheel_angle << "," << right_wheel_angle << "][" << left_wheel_angle_rate << "," << right_wheel_angle_rate << "]" << std::endl;
  }

  void sleep() {
    snooze();
  }

  double bias;
  double m_to_rad;
  double heading;
  double velocity, angular_velocity;
  double left_wheel_angle, right_wheel_angle;
  double left_wheel_angle_update, right_wheel_angle_update;
  double left_wheel_angle_rate, right_wheel_angle_rate;
  ecl::TimeStamp last_timestamp;
  ecl::Snooze snooze;


};

} // namespace kobuki

#endif /* SIMULATION_HPP_ */
