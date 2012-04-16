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
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const Battery &battery) { status = battery; }

private:
  Battery status;
};

class WatchdogTask : public diagnostic_updater::DiagnosticTask {
public:
  WatchdogTask() : DiagnosticTask("Watchdog"), alive(false) {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const bool &is_alive) { alive = is_alive; }
  bool isAlive() const { return alive; }

private:
  bool alive;
};



} // namespace kobuki

#endif /* KOBUKI_NODE_DIAGNOSTICS_HPP_ */
