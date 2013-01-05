/**
 * @file /kobuki_driver/include/kobuki_driver/modules/battery.hpp
 *
 * @brief Human friendly batter indicator class.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_BATTERY_HPP_
#define KOBUKI_BATTERY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdint.h>
#include "../packets/core_sensors.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief  Battery level module.
 *
 * Currently hard codes the battery status. It might be useful to provide
 * some configurable parameters for this module in the future.
 **/
class Battery {
public:
  enum Source {
    None,
    Adapter,
    Dock
  };
  enum Level {
    Dangerous,
    Low,
    Healthy,
    Maximum
  };
  enum State {
    Discharging,
    Charged,
    Charging
  };

  Battery() {} /**< Default constructor. **/
  Battery (const uint8_t &new_voltage, const uint8_t &charger_flag);
  Level level() const;
  float percent() const;

  static double capacity, low, dangerous;
  double voltage;
  State charging_state;
  Source charging_source;
};

} // namespace kobuki

#endif /* KOBUKI_BATTERY_HPP_ */
