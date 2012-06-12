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
