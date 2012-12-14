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

#include "kobuki_auto_docking/dock_drive.hpp"

/*****************************************************************************
** Defines
*****************************************************************************/

#define stringfy(x) #x
#define setState(x) {state=x;state_str=stringfy(x);}
#define setVel(v,w) {vx=v;wz=w;}
#define setStateVel(x,v,w) {setState(x);setVel(v,w);}

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
  , bump_remainder(0)
  , dock_stabilizer(0)
  , dock_detector(0)
  , rotated(0.0)
{
  pose.setIdentity();
}

DockDrive::~DockDrive(){;}


void DockDrive::modeShift(const std::string& mode)
{
  if (mode == "enable")  { is_enabled = true;  can_run = true; }
  if (mode == "disable") { is_enabled = false; can_run = false; }
  if (mode == "run")  can_run = true;
  if (mode == "stop") can_run = false;
}

void DockDrive::update(const std::vector<unsigned char> &signal
                , const unsigned char &bumper
                , const unsigned char &charger
                , const ecl::Pose2D<double> &pose) {
  static ecl::Pose2D<double> pose_priv;
  ecl::Pose2D<double> pose_update;

  double dx = pose.x() - pose_priv.x();
  double dy = pose.y() - pose_priv.y();
  pose_update.x( std::sqrt( dx*dx + dy*dy ) );
  pose_update.heading( pose.heading() - pose_priv.heading() );
  //std::cout << pose_diff << "=" << pose << "-" << pose_priv << " | " << pose_update << std::endl;
  pose_priv = pose;

  ecl::linear_algebra::Vector3d pose_update_rates; //dummy
  update( signal, bumper, charger, pose_update, pose_update_rates );
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
void DockDrive::update(const std::vector<unsigned char> &signal
                , const unsigned char &bumper
                , const unsigned char &charger
                , const ecl::Pose2D<double> &pose_update
                , const ecl::linear_algebra::Vector3d &pose_update_rates) {
  /*************************
   * pre processing
   *************************/
  //pose update
  pose *= pose_update;

  //dock_ir signals filtering
  past_signals.push_back(signal);
  unsigned int window = 20;
  while (past_signals.size() > window)
    past_signals.erase( past_signals.begin(), past_signals.begin() + past_signals.size() - window );

  std::vector<unsigned char> signal_filt(signal.size(), 0);
  for ( unsigned int i=0; i<past_signals.size(); i++) {
    if (signal_filt.size() != past_signals[i].size()) continue;
    for (unsigned int j=0; j<signal_filt.size(); j++)
      signal_filt[j] |= past_signals[i][j];
  }


  /*************************
   * debug prints
   *************************/
  std::ostringstream debug_stream;
  // pose
  debug_stream << pose;
  // pose_update and pose_update_rates for debugging
  debug_stream << std::fixed << std::setprecision(4)
    << "[x: "    << std::setw(7) << pose_update.x()
    << ", y: "  << std::setw(7) << pose_update.y()
    << ", th: " << std::setw(7) << pose_update.heading()
    << "]";

  //dock_ir signal
  /*
  debug_stream 
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
  debug_stream << far_signal << near_signal;

  //bumper
  {
  std::string out = "[B: ";
  if (bumper&4) out += "L"; else out += "-";
  if (bumper&2) out += "C"; else out += "-";
  if (bumper&1) out += "R"; else out += "-";
  out += "]";
  debug_stream << out;
  }

  //charger
  {
  std::ostringstream oss;
  oss << "[C:" << std::setw(2) << (unsigned int)charger;
  oss << "(";
  if (charger) oss << "ON"; else oss << "  ";
  oss << ")]";
  debug_stream << oss.str();
  }


  /*************************
   * processing. algorithms; transforma to velocity command
   *************************/
  //std::string debug_str = "";
  debug_str = "";
  do {  // a kind of hack
    if ( state==DONE ) setState(IDLE); // when this function is called after final state 'DONE'.
    if ( state==DOCKED_IN ) {
      if ( dock_stabilizer++ > 20 ) {
        is_enabled = false;
        can_run = false;
        setStateVel(DONE, 0.0, 0.0); break;
      }
      setStateVel(DOCKED_IN, 0.0, 0.0); break;
    }
    if ( bump_remainder > 0 ) {
      bump_remainder--;
      if ( charger ) { setStateVel(DOCKED_IN, 0.0, 0.0); break; } // when bumper signal is received early than charger(dock) signal.
      else {           setStateVel(BUMPED_DOCK, -0.01, 0.0); break; }
    } else if (state == BUMPED) {
      setState(IDLE); //should I remember and recall previous state?
      debug_str="how dare!!";
    }
    if ( bumper || charger ) {
      if( bumper && charger ) {
        bump_remainder = 0;
        setStateVel(BUMPED_DOCK, -0.01, 0.0); break;
      }
      if ( bumper ) {
        bump_remainder = 50;
        setStateVel(BUMPED, -0.05, 0.0); break;
      }
      if ( charger ) { // already docked in
        dock_stabilizer = 0;
        setStateVel(DOCKED_IN, 0.0, 0.0); break;
      }
    } else {
      if ( state==IDLE ) {
        dock_detector = 0;
        rotated = 0.0;
        setStateVel(SCAN, 0.00, 0.66); break;
      }
      if ( state==SCAN ) {
        rotated += pose_update.heading()/(2.0*M_PI);
        std::ostringstream oss;
        oss << "rotated: " << std::fixed << std::setprecision(2) << std::setw(4) << rotated;
        debug_str = oss.str();
        if( std::abs(rotated) > 1.6 ) {
          setStateVel(FIND_STREAM, 0.0, 0.0); break;
        }
        if (  signal_filt[1]&(FAR_LEFT  + NEAR_LEFT )) dock_detector--;
        if (  signal_filt[1]&(FAR_RIGHT + NEAR_RIGHT)) dock_detector++;
        if ( (signal_filt[1]&FAR_CENTER) || (signal_filt[1]&NEAR_CENTER) ) {
          setStateVel(ALIGNED, 0.05, 0.00); break;
        } else if ( signal_filt[1] ) {
          setStateVel(SCAN, 0.00, 0.10); break;
        } else {
          setStateVel(SCAN, 0.00, 0.66); break;
        }

      } else if (state==ALIGNED || state==ALIGNED_FAR || state==ALIGNED_NEAR) {
        if ( signal_filt[1] ) {
          if ( signal_filt[1]&NEAR )
          {
            if ( ((signal_filt[1]&NEAR) == NEAR_CENTER) || ((signal_filt[1]&NEAR) == NEAR) ) { setStateVel(ALIGNED_NEAR, 0.05,  0.0); debug_str = "AlignedNearCenter"; break; }
            if (   signal_filt[1]&NEAR_LEFT  ) {                                               setStateVel(ALIGNED_NEAR, 0.05,  0.1); debug_str = "AlignedNearLeft"  ; break; }
            if (   signal_filt[1]&NEAR_RIGHT ) {                                               setStateVel(ALIGNED_NEAR, 0.05, -0.1); debug_str = "AlignedNearRight" ; break; }
          }
          if ( signal_filt[1]&FAR )
          {
            if ( ((signal_filt[1]&FAR) == FAR_CENTER) || ((signal_filt[1]&FAR) == FAR) ) { setStateVel(ALIGNED_FAR, 0.1,  0.0); debug_str = "AlignedFarCenter"; break; }
            if (   signal_filt[1]&FAR_LEFT  ) {                                            setStateVel(ALIGNED_FAR, 0.1,  0.3); debug_str = "AlignedFarLeft"  ; break; }
            if (   signal_filt[1]&FAR_RIGHT ) {                                            setStateVel(ALIGNED_FAR, 0.1, -0.3); debug_str = "AlignedFarRight" ; break; }
          }
          dock_detector = 0;
          rotated = 0.0;
          setStateVel(SCAN, 0.00, 0.66); break;
        } else {
          debug_str = "lost signals";
          setStateVel(LOST, 0.00, 0.00); break;
        }
      } else if (state==FIND_STREAM) {
        if (dock_detector > 0 ) { // robot is placed in right side of docking station
          //turn  right , negative direction til get right signal from left sensor
          if (signal_filt[2]&(FAR_RIGHT+NEAR_RIGHT)) {
            setStateVel(GET_STREAM, 0.05, 0.0); break;
          } else {
            setStateVel(FIND_STREAM, 0.0, -0.33); break;
          }
        } else if (dock_detector < 0 ) { // robot is placed in left side of docking station
          //turn left, positive direction till get left signal from right sensor
          if (signal_filt[0]&(FAR_LEFT+NEAR_LEFT)) {
            setStateVel(GET_STREAM, 0.05, 0.0); break;
          } else {
            setStateVel(FIND_STREAM, 0.0, 0.33); break;
          }
        }
      } else if (state==GET_STREAM) {
        if (dock_detector > 0) { //robot is placed in right side of docking station
          if (signal_filt[2]&(FAR_LEFT+NEAR_LEFT)) {
            dock_detector = 0;
            rotated = 0.0;
            setStateVel(SCAN, 0.0, 0.10); break;
          } else {
            setStateVel(GET_STREAM, 0.05, 0.0); break;
          }
        } else if (dock_detector < 0) { // robot is placed in left side of docking station
          if (signal_filt[0]&(FAR_RIGHT+NEAR_RIGHT)) {
            dock_detector = 0;
            rotated = 0.0;
            setStateVel(SCAN, 0.0, 0.10); break;
          } else {
            setStateVel(GET_STREAM, 0.05, 0.0); break;
          }
        }
      } else {
        dock_detector = 0;
        rotated = 0.0;
        setStateVel(SCAN, 0.00, 0.66); break;
      }
    }
    setStateVel(UNKNOWN, 0.00, 0.00); break;
  } while(0);

  //debug_stream << std::fixed << std::setprecision(4)
  debug_stream << "[vx: " << std::setw(7) << vx << ", wz: " << std::setw(7) << wz << "]";
  debug_stream << "[S: " << state_str << "]";
  debug_stream << "[" << debug_str << "]";
  //debug_stream << std::endl;
  debug_output = debug_stream.str();

  //std::cout << debug_output << std::endl;;

  velocityCommands(vx, wz);
//  this->vx = vx; this->wz = wz;
  return;
}

void DockDrive::velocityCommands(const double &vx_, const double &wz_) {
  // vx: in m/s
  // wz: in rad/s
  vx = vx_;
  wz = wz_;
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
