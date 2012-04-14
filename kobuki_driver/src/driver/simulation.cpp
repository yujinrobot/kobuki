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
 * @file /kobuki_node/src/node/simulation.cpp
 *
 * @brief Simulation implementation
 *
 * @date 15/04/2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/kobuki_driver/simulation.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

Simulation::Simulation() :
  heading(0.0),
  velocity(0.0),
  angular_velocity(0.0),
  left_wheel_angle(0.0),
  right_wheel_angle(0.0),
  left_wheel_angle_update(0.0),
  right_wheel_angle_update(0.0),
  left_wheel_angle_rate(0.0),
  right_wheel_angle_rate(0.0),
  is_simulation(false),
  snooze(ecl::Duration(0.02))
  {
  }

void Simulation::init(const double &b, const double &metres_to_radians) {
  bias = b;
  m_to_rad = metres_to_radians;
  is_simulation = true;
}

void Simulation::reset() {
  left_wheel_angle = 0.0;
  right_wheel_angle = 0.0;
  left_wheel_angle_rate = 0.0;
  right_wheel_angle_rate = 0.0;
  last_timestamp.stamp();
  heading = 0.0;
}

void Simulation::update() {
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

  // debugUpdate();
}

void Simulation::debugUpdate() const {
  std::cout << "Update:" << std::endl;
  std::cout << "  timestamp: " << last_timestamp << std::endl;
  std::cout << "  velocity: " << velocity << std::endl;
  std::cout << "  angular rate: " << angular_velocity << std::endl;
  std::cout << "  wheel_angle: [" << left_wheel_angle << "," << right_wheel_angle << "]" << std::endl;
  std::cout << "  wheel_angle_rate: [" << left_wheel_angle_rate << "," << right_wheel_angle_rate << "]" << std::endl;
  std::cout << "  heading: " << heading << std::endl;
}


} // namespace kobuki
