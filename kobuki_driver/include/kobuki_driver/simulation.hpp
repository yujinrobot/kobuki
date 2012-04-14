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

#include <ecl/time/snooze.hpp>
#include <ecl/time/timestamp.hpp>
#include <ecl/geometry/angle.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class Simulation {
public:
  Simulation();
  void init(const double &b, const double &metres_to_radians);
  void reset();
  void update();
  void debugUpdate() const;
  void sleep() { snooze(); }
  bool operator()() const { return is_simulation; }

  double bias;
  double m_to_rad;
  ecl::Angle<double> heading;
  double velocity, angular_velocity;
  double left_wheel_angle, right_wheel_angle;
  double left_wheel_angle_update, right_wheel_angle_update;
  double left_wheel_angle_rate, right_wheel_angle_rate;
  ecl::TimeStamp last_timestamp;

private:
  bool is_simulation;
  ecl::Snooze snooze;

};

} // namespace kobuki

#endif /* SIMULATION_HPP_ */
