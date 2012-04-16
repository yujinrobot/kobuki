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
}

void WatchdogTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( alive ) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Alive");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Signal");
  }
}

} // namespace kobuki
