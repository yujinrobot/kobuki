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
