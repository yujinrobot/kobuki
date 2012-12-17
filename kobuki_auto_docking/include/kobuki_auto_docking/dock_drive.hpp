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
 * @file /kobuki_driver/include/kobuki_driver/modules/dock_drive.hpp
 *
 * @brief Simple module for the docking drive algorithm
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_DOCK_DRIVE_HPP_
#define KOBUKI_DOCK_DRIVE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <ecl/geometry/pose2d.hpp>
#include <ecl/linear_algebra.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class DockDrive {
public:
  enum Station {
    NEAR_LEFT=1,
    NEAR_CENTER=2,
    NEAR_RIGHT=4,
    FAR_CENTER=8,
    FAR_LEFT=16,
    FAR_RIGHT=32,
    NEAR = 7,
    FAR = 56,
  };
  enum State {
    IDLE,
    LOST,
    UNKNOWN,
    INSIDE_FIELD,
    AWAY,
    SCAN,
    SPIN,
    SPIRAL,
    FIND_STREAM,
    GET_STREAM,
    ALIGNED,
    ALIGNED_FAR,
    ALIGNED_NEAR,
    BUMPED,
    BUMPED_DOCK,
    RUN,
    STOP,
    DOCKED_IN,
    DONE,
  };

  DockDrive();
  ~DockDrive();

  bool init(){ return true; }
  bool isEnabled() const { return is_enabled; }
  bool canRun() const { return can_run; }

  void enable() { modeShift("enable"); }
  void disable() { modeShift("disable"); }
  void modeShift(const std::string& mode);

  void update(const std::vector<unsigned char> &signal /* dock_ir signal*/
                , const unsigned char &bumper
                , const unsigned char &charger
                , const ecl::Pose2D<double> &pose);

  void update(const std::vector<unsigned char> &signal /* dock_ir signal*/
                , const unsigned char &bumper
                , const unsigned char &charger
                , const ecl::Pose2D<double> &pose_update
                , const ecl::linear_algebra::Vector3d &pose_update_rates);

  void velocityCommands(const double &vx, const double &wz);

  /*********************
  ** Command Accessors
  **********************/
  double getVX() const { return vx; }
  double getWZ() const { return wz; }

  /*********************
  ** Mode Accessors
  **********************/
  State getState() const { return state; }
  std::string getStateStr() const { return state_str; }
  std::string getDebugStr() const { return debug_str; }

  //debugging
  std::string getDebugStream() { return debug_output; } //stream.str(); }
  //std::string getDebugStream() { return debug_stream.str(); }
  //std::ostringstream debug_stream;

private:
  bool is_enabled, can_run;

  State state;
  std::string state_str, debug_str;
  ecl::Pose2D<double> pose;
  double vx, wz;
  std::vector<std::vector<unsigned char> > past_signals;
  int bump_remainder;
  int dock_stabilizer;
  int dock_detector;
  double rotated;

  std::string binary(unsigned char number) const;

  std::string debug_output;
};

} // namespace kobuki

#endif /* KOBUKI_DOCK_DRIVE_HPP_ */
