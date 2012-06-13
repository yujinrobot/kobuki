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
 * @file /kobuki_node/src/node/diagnostics.cpp
 *
 * @brief Robot diagnostics implementation.
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_node/diagnostics.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

void BatteryTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  switch ( status.level() ) {
    case ( Battery::Maximum ) : {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Maximum");
      break;
    }
    case ( Battery::Healthy ) : {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Healthy");
      break;
    }
    case ( Battery::Low ) : {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Low");
      break;
    }
    case ( Battery::Dangerous ) : {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Dangerous");
      break;
    }
  }
  stat.add("Capacity", status.capacity);
  stat.add("Voltage", status.voltage);
  stat.add("Percent", status.percent());
  switch (status.charging_source ) {
    case(Battery::None) : {
      stat.add("Source", "None");
      break;
    }
    case(Battery::Adapter) : {
      stat.add("Source", "Adapter");
      break;
    }
    case(Battery::Dock) : {
      stat.add("Source", "Dock");
      break;
    }
  }
  switch ( status.charging_state ) {
    case ( Battery::Charged ) : {
      stat.add("State", "Charged");
      break;
    }
    case ( Battery::Charging ) : {
      stat.add("State", "Charging");
      break;
    }
    case ( Battery::Discharging ) : {
      stat.add("State", "Discharging");
      break;
    }
    default: break;
  }
}

void WatchdogTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( alive ) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Alive");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Signal");
  }
}

void CliffSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( status ) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Cliff Detected!");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   "Reading: %d  Cliff: %s", values.bottom[0], status & CoreSensors::Flags::LeftCliff?"YES":"NO");
  stat.addf("Center", "Reading: %d  Cliff: %s", values.bottom[1], status & CoreSensors::Flags::CenterCliff?"YES":"NO");
  stat.addf("Right",  "Reading: %d  Cliff: %s", values.bottom[2], status & CoreSensors::Flags::RightCliff?"YES":"NO");
}

void WallSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( status ) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Wall Hit!");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   status & CoreSensors::Flags::LeftBumper?"YES":"NO");
  stat.addf("Center", status & CoreSensors::Flags::CenterBumper?"YES":"NO");
  stat.addf("Right",  status & CoreSensors::Flags::RightBumper?"YES":"NO");
}

void WheelDropTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( status ) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Wheel Drop!");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   status & CoreSensors::Flags::LeftWheel?"YES":"NO");
  stat.addf("Right",  status & CoreSensors::Flags::RightWheel?"YES":"NO");
}

void MotorCurrentTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( std::max(values[0], values[1]) > 6 ) { // TODO not sure about this threshold; should be a parameter?
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Is robot stalled? Motors current is very high");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",  "%d", values[0]);
  stat.addf("Right", "%d", values[1]);
}

void GyroSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  // Raw data angles are in hundredths of degree
  stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Heading: %.2f degrees", heading/100.0);
}

void DigitalInputTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "[%d, %d, %d, %d]",
                status & 0x08?1:0, status & 0x04?1:0,
                status & 0x02?1:0, status & 0x01?1:0);
}

void AnalogInputTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "[%d, %d, %d, %d]",
                values[0], values[1], values[2], values[3]);
}

} // namespace kobuki
