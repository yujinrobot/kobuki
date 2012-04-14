/**
 * @file /kobuki_node/include/kobuki_node/diagnostics.hpp
 *
 * @brief Diagnostics for the kobuki node.
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_NODE_DIAGNOSTICS_HPP_
#define KOBUKI_NODE_DIAGNOSTICS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <kobuki_driver/modules/battery.hpp>
#include <diagnostic_updater/diagnostic_updater.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class BatteryTask : public diagnostic_updater::DiagnosticTask {
public:
  BatteryTask() : DiagnosticTask("Battery") {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
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

  void update(const Battery &battery) { status = battery; }

private:
  Battery status;
};

} // namespace kobuki

#endif /* KOBUKI_NODE_DIAGNOSTICS_HPP_ */
