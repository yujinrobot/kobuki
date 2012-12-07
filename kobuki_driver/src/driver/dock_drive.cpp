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
** Defines
*****************************************************************************/

#define stringfy(x) #x
#define setState(x) {state=x; state_str=stringfy(x);}
#define setVel(v,w) {vx=x; wz=w;}
#define setAll(x,v,w) { setState(x); setVel(v,w); }

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/
DockDrive::DockDrive() :
  is_enabled(false), can_run(false) 
  , state(IDLE), state_str("IDLE")
  , vx(0.0), wz(0.0)
  , speed(0), radius(0)
  , bias(0.23) //wheelbase, wheel_to_wheel, in [m]
  , bump_remainder(0)
  , dock_stabilizer(0)
{}


void DockDrive::mode_shift(std::string mode)
{
  if (mode == "enable")  { is_enabled = true;  can_run = false; }
  if (mode == "disable") { is_enabled = false; can_run = false; }
  if (mode == "run")  can_run = true;
  if (mode == "stop") can_run = false;
}


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
void DockDrive::update(std::vector<unsigned char> &signal
                , unsigned char &bumper
                , unsigned char &charger
                , ecl::Pose2D<double> &pose_update  
                , ecl::linear_algebra::Vector3d &pose_update_rates) {
  /*************************
   * pre processing
   *************************/
  //dock_ir signals filtering
  past_signals.push_back(signal);
  while (past_signals.size() > 10) past_signals.erase( past_signals.begin(), past_signals.begin() + past_signals.size() - 10) ;

  std::vector<unsigned char> signal_filt(signal.size(), 0);
  for ( unsigned int i=0; i<past_signals.size(); i++) {
    if (signal_filt.size() != past_signals[i].size()) continue;
    for (unsigned int j=0; j<signal_filt.size(); j++)
      signal_filt[j] |= past_signals[i][j];
  }


  /*************************
   * debug prints
   *************************/
  // pose_update and pose_update_rates for debugging
  std::cout << std::fixed << std::setprecision(4)
    << "[x: "    << std::setw(7) << pose_update.x()
    << ", y: "  << std::setw(7) << pose_update.y()
    << ", th: " << std::setw(7) << pose_update.heading()
    << "]";

  //dock_ir signal
  /*
  std::cout 
    << "[l: "  << binary(signal_filt[2])
    << ", c: " << binary(signal_filt[1])
    << ", r: " << binary(signal_filt[0])
    << "]";
  */
  std::string far_signal  = "[F: "; //far field
  std::string near_signal = "[N: "; //near field
  for (unsigned int i=0; i<3; i++) {
    if (signal_filt[2-i]&FAR_LEFT   ) far_signal  += "L"; else far_signal  += "-";
    if (signal_filt[2-i]&FAR_CENTER ) far_signal  += "C"; else far_signal  += "-";
    if (signal_filt[2-i]&FAR_RIGHT  ) far_signal  += "R"; else far_signal  += "-";
    if (signal_filt[2-i]&NEAR_LEFT  ) near_signal += "L"; else near_signal += "-";
    if (signal_filt[2-i]&NEAR_CENTER) near_signal += "C"; else near_signal += "-";
    if (signal_filt[2-i]&NEAR_RIGHT ) near_signal += "R"; else near_signal += "-";
    far_signal  += " ";
    near_signal += " ";
  }
  far_signal  += "]";
  near_signal += "]";
  std::cout << far_signal << near_signal;

  //bumper
  {
  std::string out = "[B: ";
  if (bumper&4) out += "L"; else out += "-";
  if (bumper&2) out += "C"; else out += "-";
  if (bumper&1) out += "R"; else out += "-";
  out += "]";
  std::cout << out;
  }

  //charger
  {
  std::ostringstream oss;
  oss << "[C:" << std::setw(2) << (unsigned int)charger;
  oss << "(";
  if (charger) oss << "ON"; else oss << "  ";
  oss << ")]";
  std::cout << oss.str();
  }


  /*************************
   * processing. algorithms; transforma to velocity command
   *************************/
#if 0
  // just go forward
  vx = 0.1;
  wz = 0.0;

  std::cout << "[vx: " << vx << ", wz: " << wz << "]";
  std::cout << td::endl;
  velocityCommands(vx, wz);
  return;
#elif 1
  std::string debug_str = "";
  do {  // a kind of hack
    if ( state==DONE ) setState(IDLE);
    if ( state==DOCKED_IN ) {
      if (dock_stabilizer++ > 20 ) {
        is_enabled = false;
        can_run = false;
        setState(DONE); vx = 0.0; wz = 0.0; break;
      }
      setState(DOCKED_IN); vx = 0.0; wz = 0.0; break;
    }
    if ( bump_remainder > 0 ) {
      bump_remainder--;
      setState(BUMPED);
      vx = -0.05; wz = 0.0;
      break;
    } else if (state == BUMPED) {
      setState(IDLE);
    }
    if ( bumper || charger ) {
      if( bumper && charger ) {
        setState(BUMPED_DOCK);
        bump_remainder = 0;
        vx = -0.01; wz = 0.0;
        break;
      }
      if ( bumper ) {
        setState(BUMPED);
        bump_remainder = 50;
        vx = -0.05; wz = 0.0;
        break;
      }
      if ( charger ) { // already docked in
        vx = 0.0; wz = 0.0;
        setState(DOCKED_IN);
        dock_stabilizer = 0;
        break;
      }
    } else {
      if (state==SCAN || state==IDLE) {
        if ( (signal_filt[1]&FAR_CENTER) || (signal_filt[1]&NEAR_CENTER) ) {
          setState(ALIGNED); vx = 0.05; wz = 0.00; break;
        } else if ( signal_filt[1] ) {
          setState(SCAN);    vx = 0.00; wz = 0.10; break;
        } else {
          setState(SCAN);    vx = 0.00; wz = 0.66; break;
        }

      } else if (state==ALIGNED || state==ALIGNED_FAR || state==ALIGNED_NEAR) {
        if ( signal_filt[1] ) {
          if (signal_filt[1]&NEAR)
          {
            if( ((signal_filt[1]&NEAR) == NEAR_CENTER) || ((signal_filt[1]&NEAR) == NEAR) ) { setState(ALIGNED_NEAR); vx = 0.05; wz = 0.00; debug_str = "AlignedNearCenter"; break; }
            if(   signal_filt[1]&NEAR_LEFT  ) {                                               setState(ALIGNED_NEAR); vx = 0.05; wz = 0.1;  debug_str = "AlignedNearLeft"  ; break; }
            if(   signal_filt[1]&NEAR_RIGHT ) {                                               setState(ALIGNED_NEAR); vx = 0.05; wz = -0.1; debug_str = "AlignedNearRight" ; break; }
          }
          if (signal_filt[1]&FAR)
          {
            if( ((signal_filt[1]&FAR) == FAR_CENTER) || ((signal_filt[1]&FAR) == FAR) ) { setState(ALIGNED_FAR);  vx = 0.1;  wz = 0.00; debug_str = "AlignedFarCenter"; break; }
            if(   signal_filt[1]&FAR_LEFT ) {                                             setState(ALIGNED_FAR);  vx = 0.1;  wz = 0.3;  debug_str = "AlignedFarLeft"  ; break; }
            if(   signal_filt[1]&FAR_RIGHT ) {                                            setState(ALIGNED_FAR);  vx = 0.1;  wz = -0.3; debug_str = "AlignedFarRight" ; break; }
          } else {                                                                        setState(SCAN);         vx = 0.00; wz = 0.66; break; }
        } else {                                                                          setState(SCAN);         vx = 0.00; wz = 0.66; break; }
      } else {                                                                            setState(UNKNOWN);      vx = 0.00; wz = 0.00; break; }
    } /*else {
      setState(IDLE);
    }*/

  } while(0);

  //std::cout << std::fixed << std::setprecision(4)
  std::cout << "[vx: " << std::setw(7) << vx << ", wz: " << std::setw(7) << wz << "]";
  std::cout << "[S: " << state_str << "]";
  std::cout << "[" << debug_str << "]";
  std::cout << std::endl;
  velocityCommands(vx, wz);
  return;
#elif 0
  if (signal[0]==0 && signal[1]==0 && signal[2]==0) {
    vx = 0.0; wz = 0.33;
    std::cout << " vx: " << vx << ", wz: " << wz << std::endl;
    velocityCommands(vx, wz);
    return;
  } // if there are no sensor inputs

  if ((signal[0]!=0 && signal[1]!=0)
      || (signal[1]!=0 && signal[2]!=0)
      || (signal[2]!=0 && signal[0]!=0)){
    std::cout << "it is weird cases. #001 - multiple signal sources" << std::endl;
    return; //without any update
  } // if there are multiple input (in case dock_ir signals from multiple sources, or by reflections)
  // do not consider it for now
  if (signal[1] != 0) {
    if (signal[1]&FAR_CENTER) { 
      vx = 0.1; wz =  0.00; 
      std::cout << " vx: " << vx << ", wz: " << wz << std::endl;
      velocityCommands(vx, wz);
      return; 
    } else {
      if (signal[1]&FAR_LEFT && signal[1]&FAR_RIGHT ) { 
        vx = 0.1; wz = 0.00; 
        std::cout << " vx: " << vx << ", wz: " << wz << std::endl;
        velocityCommands(vx, wz);
        std::cout << "weird cases. #002" << std::endl; 
        return;
      }
      if (signal[1]&FAR_LEFT  ) { 
        vx = 0.1; wz = +0.3; 
        std::cout << " vx: " << vx << ", wz: " << wz << std::endl;
        velocityCommands(vx, wz);
        return; 
      }
      if (signal[1]&FAR_RIGHT ) { 
        vx = 0.1; wz = -0.3; 
        std::cout << " vx: " << vx << ", wz: " << wz << std::endl;
        velocityCommands(vx, wz);
        return; 
      }
    }
  }
  if (signal[0] != 0) {
    if (signal[0]&FAR_CENTER) { 
      vx = 0.1; wz =  0.00; 
      std::cout << " vx: " << vx << ", wz: " << wz << std::endl;
      velocityCommands(vx, wz);
      return; 
    } // most simplar case
    return;
  }
  if (signal[2] != 0) {
    if (signal[1]&FAR_CENTER) { 
      vx = 0.1; wz =  0.00; 
      std::cout << " vx: " << vx << ", wz: " << wz << std::endl;
      velocityCommands(vx, wz);
      return; 
    }
    return;
  } 
  /*
    if (signal[0] != 0) {
      if (signal[0]^FAR_CENTER) { vx = 0.1; wz =  0.00; }
      if (signal[0]^FAR_LEFT  ) { vx = 0.1; wz =  0.00; }
      if (signal[0]^FAR_RIGHT ) { vx = 0.1; wz =  0.00; }
      return;
    }
    if (signal[2] != 0) {
      if (signal[2]^FAR_CENTER) { vx = 0.1; wz =  0.00; }
      if (signal[2]^FAR_LEFT  ) { vx = 0.1; wz =  0.00; }
      if (signal[2]^FAR_RIGHT ) { vx = 0.1; wz =  0.00; }
      return;
    }
    std::cout << "never reachable here. #003" << std::endl;
    vx=0.0; wz=0.0;
  */
  vx=0.0; wz=0.0;
  std::cout << " vx: " << vx << ", wz: " << wz << std::endl;
  velocityCommands(vx, wz);
  std::cout << "never reachable here. #002" << std::endl;
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

std::string DockDrive::binary(unsigned char number) const {
  std::string ret;
  for( unsigned int i=0;i<6; i++){
    if (number&1) ret = "1" + ret;
    else          ret = "0" + ret;
    number = number >> 1;
  }
  return ret;
}


} // namespace kobuki
