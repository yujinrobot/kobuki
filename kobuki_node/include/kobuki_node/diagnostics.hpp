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
  BatteryTask() : DiagnosticTask("Battery diagnostics"), count(0) {}
  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Print warnings");
    stat.add("Counter", count);
    ++count;
  }
private:
  int count;
};

} // namespace kobuki

#endif /* KOBUKI_NODE_DIAGNOSTICS_HPP_ */
