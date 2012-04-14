/**
 * @file /kobuki_driver/src/driver/battery.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 15/04/2012
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

uint8_t Battery::capacity = 170;

/*****************************************************************************
** Implementation
*****************************************************************************/

Battery::Battery (const uint8_t &new_voltage, const uint8_t &charger_flag) :
  voltage(new_voltage)
{
  if ( charger_flag & CoreSensors::Flags::Adapter ) {
    charging_source = Adapter;
  } else if ( charger_flag & CoreSensors::Flags::Dock ) {
    charging_source = Dock;
  } else {
    charging_source = None;
  }
  // add a check here to modify the capacity if the incoming charger flag
  // is set to Adapter or dock, and the flag also indicates that it is maxed
  // i.e. set capacity == voltage_level in this case
};

} // namespace kobuki
