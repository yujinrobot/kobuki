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
 * @brief Battery diagnostics implementation.
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
  stat.add("Capacity", static_cast<int>(status.capacity));
  stat.add("Voltage", static_cast<int>(status.voltage));
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
      stat.add("State", "Discharged");
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

} // namespace kobuki
