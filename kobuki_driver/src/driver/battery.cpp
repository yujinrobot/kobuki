/**
 * @file /kobuki_driver/src/driver/battery.cpp
 *
 * @brief Battery/charging source implementation
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/modules/battery.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Statics
*****************************************************************************/

double Battery::capacity  = 16.5;
double Battery::low       = 14.0;
double Battery::dangerous = 13.2;

/*****************************************************************************
** Implementation
*****************************************************************************/
/**
 * Configures the battery status given the current sensor readings.
 *
 * @param new_voltage : measured voltage*10
 * @param charger_flag : bit flag representing charging status and source
 */
Battery::Battery (const uint8_t &new_voltage, const uint8_t &charger_flag) :
  voltage(static_cast<double>(new_voltage)/10.0)
{
  uint8_t state = (charger_flag & CoreSensors::Flags::BatteryStateMask);
  if ( state == CoreSensors::Flags::Charging) {
    charging_state = Charging;
  } else if ( state == CoreSensors::Flags::Charged ) {
    charging_state = Charged;
    capacity = voltage;
  } else {
    charging_state = Discharging;
  }

  if (charging_state == Discharging) {
    charging_source = None;
  } else if ( charger_flag & CoreSensors::Flags::AdapterType ) {
    charging_source = Adapter;
  } else {
    charging_source = Dock;
  }
};

Battery::Level Battery::level() const {
  if ( charging_state == Charged ) { return Maximum; }
  if ( voltage > low ) { return Healthy; }
  if ( voltage > dangerous ) { return Low; }
  return Dangerous;
}

float Battery::percent() const {
  // convert battery voltage to percent: 100% -> capacity / 5% -> dangerous
  float percent = ((95*(voltage - dangerous)) / (capacity - dangerous)) + 5;
  return std::max(std::min(percent, 100.0f), 0.0f);
}

} // namespace kobuki
