/**
 * @file /kobuki_driver/include/kobuki_driver/modules/battery.hpp
 *
 * @brief Human friendly batter indicator class.
 *
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
  Battery() {} /**< Default constructor. **/
  Battery (const uint8_t &new_voltage, const uint8_t &charger_flag);
  Level level() const {
    if ( voltage == capacity ) { return Maximum; }
    float remaining = static_cast<float>(voltage)/static_cast<float>(capacity);
    const float healthy = 0.9;
    const float low = 0.85;
    if ( remaining > healthy ) { return Healthy; }
    if ( remaining > low ) { return Low; }
    return Dangerous;
  }

  static uint8_t capacity;
  uint8_t voltage;
  Source charging_source;
};

} // namespace kobuki

#endif /* KOBUKI_BATTERY_HPP_ */
