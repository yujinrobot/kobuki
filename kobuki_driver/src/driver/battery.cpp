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
 * @file /kobuki_driver/src/driver/battery.cpp
 *
 * @brief Battery/charging source implementation
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
uint8_t Battery::low = 135;
uint8_t Battery::dangerous = 131;

/*****************************************************************************
** Implementation
*****************************************************************************/

Battery::Battery (const uint8_t &new_voltage, const uint8_t &charger_flag) :
  voltage(new_voltage)
{
  uint8_t state = (charger_flag & CoreSensors::Flags::BatteryStateMask);
  if ( state == CoreSensors::Flags::Charging) {
    charging_state = Charging;
  } else if ( state == CoreSensors::Flags::Charged ) {
    charging_state = Charged;
    capacity = new_voltage;
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

} // namespace kobuki
