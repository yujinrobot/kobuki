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
 * @file /kobuki_driver/src/driver/dock_drive.cpp
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/modules/dock_drive.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/
DockDrive::DockDrive() :
  vx(0.0), wz(0.0)
  , speed(0), radius(0)
  , bias(0.23) //wheelbase, wheel_to_wheel, in [m]
{}


/**
 * @brief Updates the odometry from firmware stamps and encoders.
 *
 * Really horrible - could do with an overhaul.
 *
 * @param time_stamp
 * @param left_encoder
 * @param right_encoder
 * @param pose_update
 * @param pose_update_rates
 */
void DockDrive::update(std::vector<unsigned char> &dock_ir
                , ecl::Pose2D<double> &pose_update  
                , ecl::linear_algebra::Vector3d &pose_update_rates) {
  // pose_update and pose_update_rates for debugging

  // Do Something!!!
  std::cout << "x: " << pose_update.x()
    << ", y: " << pose_update.y()
    << ", th: " << pose_update.heading() 
    << std::endl;
  std::vector<std::string> bin(3, "");
  for( int i=0;i<3; i++){
    unsigned int number = (unsigned int)dock_ir[i];
  for( int j=0;j<6; j++){
    //std::cout << number << std::endl;
    if( ( number & 1 ) == 0 )
      bin[i] += "0";
    else
      bin[i] += "1";
    number = number >> 1;
  } 
  }
  std::cout 
    << "l: "   << (unsigned)dock_ir[2]
    << ", c: " << (unsigned)dock_ir[1]
    << ", r: " << (unsigned)dock_ir[0]   
    << std::endl;
  std::cout 
    << "l: "   << bin[2]
    << ", c: " << bin[1]
    << ", r: " << bin[0]   
    << std::endl;
  std::cout << "--------------------------------------------------------------------------------" << std::endl;

//  dock_ir_history.push_back(dock_ir);

#if 0   
  // do nothing
  vx = 0.0;
  wz = 0.0;
#else
  if (dock_ir[0]==0 && dock_ir[1]==0 && dock_ir[2]==0) {
    vx = 0.0; wz = 0.33;
    velocityCommands(vx, wz);
    return;
  } // if there are no sensor inputs

  if ((dock_ir[0]!=0 && dock_ir[1]!=0)
      || (dock_ir[1]!=0 && dock_ir[2]!=0)
      || (dock_ir[2]!=0 && dock_ir[0]!=0)){
    std::cout << "it is weird cases. #001 - multiple signal sources" << std::endl;
    return; //without any update
  } // if there are multiple input (in case dock_ir signals from multiple sources, or by reflections)
  // do not consider it for now
  if (dock_ir[1] != 0) {
    if (dock_ir[1]&FAR_CENTER) { 
      vx = 0.1; wz =  0.00; 
      velocityCommands(vx, wz);
      return; 
    } else {
      if (dock_ir[1]&FAR_LEFT && dock_ir[1]&FAR_RIGHT ) { 
        vx = 0.1; wz = 0.00; 
        velocityCommands(vx, wz);
        std::cout << "weird cases. #002" << std::endl; 
        return;
      }
      if (dock_ir[1]&FAR_LEFT  ) { 
        vx = 0.1; wz = +0.3; 
        velocityCommands(vx, wz);
        return; 
      }
      if (dock_ir[1]&FAR_RIGHT ) { 
        vx = 0.1; wz = -0.3; 
        velocityCommands(vx, wz);
        return; 
      }
    }
  }
  if (dock_ir[0] != 0) {
    if (dock_ir[0]&FAR_CENTER) { 
      vx = 0.1; wz =  0.00; 
      velocityCommands(vx, wz);
      return; 
    } // most simplar case
    return;
  }
  if (dock_ir[2] != 0) {
    if (dock_ir[1]&FAR_CENTER) { 
      vx = 0.1; wz =  0.00; 
      velocityCommands(vx, wz);
      return; 
    }
    return;
  } 
  /*
    if (dock_ir[0] != 0) {
      if (dock_ir[0]^FAR_CENTER) { vx = 0.1; wz =  0.00; }
      if (dock_ir[0]^FAR_LEFT  ) { vx = 0.1; wz =  0.00; }
      if (dock_ir[0]^FAR_RIGHT ) { vx = 0.1; wz =  0.00; }
      return;
    }
    if (dock_ir[2] != 0) {
      if (dock_ir[2]^FAR_CENTER) { vx = 0.1; wz =  0.00; }
      if (dock_ir[2]^FAR_LEFT  ) { vx = 0.1; wz =  0.00; }
      if (dock_ir[2]^FAR_RIGHT ) { vx = 0.1; wz =  0.00; }
      return;
    }
    std::cout << "never reachable here. #003" << std::endl;
    vx=0.0; wz=0.0;
  */
  std::cout << "never reachable here. #002" << std::endl;
  vx=0.0; wz=0.0;
  velocityCommands(vx, wz);
  return;
#endif

}

void DockDrive::velocityCommands(const double &vx, const double &wz) {
  // vx: in m/s
  // wz: in rad/s
  const double epsilon = 0.0001;
  if ( std::abs(wz) < epsilon ) {
    radius = 0;
  } else if ( (std::abs(vx) < epsilon ) && ( wz > epsilon ) ) {
    radius = 1;
  } else if ( (std::abs(vx) < epsilon ) && ( wz < -1*epsilon ) ) {
    radius = -1;
  } else {
    radius = (short)(vx * 1000.0f / wz);
  }
  if ( vx < 0.0 ) {
    speed = (short)(1000.0f * std::min(vx + bias * wz / 2.0f, vx - bias * wz / 2.0f));
  } else {
    speed = (short)(1000.0f * std::max(vx + bias * wz / 2.0f, vx - bias * wz / 2.0f));
  }
}

std::vector<short> DockDrive::velocityCommands() const {
  std::vector<short> cmd(2);
  cmd[0] = speed;
  cmd[1] = radius;
  return cmd;
}

} // namespace kobuki
