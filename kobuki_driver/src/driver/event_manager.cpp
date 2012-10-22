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
 * @file /kobuki_driver/src/driver/event_manager.cpp
 *
 * @brief Implementation of the event black magic.
 *
 * @date 14/04/2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/event_manager.hpp"
#include "../../include/kobuki_driver/modules/battery.hpp"
#include "../../include/kobuki_driver/packets/core_sensors.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

void EventManager::init ( const std::string &sigslots_namespace ) {
  sig_button_event.connect(sigslots_namespace + std::string("/button_event"));
  sig_bumper_event.connect(sigslots_namespace + std::string("/bumper_event"));
  sig_cliff_event.connect(sigslots_namespace  + std::string("/cliff_event"));
  sig_wheel_event.connect(sigslots_namespace  + std::string("/wheel_event"));
  sig_power_event.connect(sigslots_namespace  + std::string("/power_event"));
  sig_input_event.connect(sigslots_namespace  + std::string("/input_event"));
}

/**
 * Update with incoming data and emit events if necessary.
 * @param new_state  Updated core sensors state
 * @param cliff_data Cliff sensors readings (we include them as an extra information on cliff events)
 */
void EventManager::update(const CoreSensors::Data &new_state, const std::vector<uint16_t> &cliff_data) {
  if (last_state.buttons != new_state.buttons)
  {
    // ------------
    // Button Event
    // ------------

    // Note that the touch pad means at most one button can be pressed
    // at a time.
    ButtonEvent event;

    // Check changes in each button state's; even if this block of code
    // supports it, two buttons cannot be pressed simultaneously
    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::B0) {
      event.button = ButtonEvent::B0;
      if (new_state.buttons & CoreSensors::Flags::B0) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }

    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::B1) {
      event.button = ButtonEvent::B1;
      if (new_state.buttons & CoreSensors::Flags::B1) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }

    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::B2) {
      event.button = ButtonEvent::B2;
      if (new_state.buttons & CoreSensors::Flags::B2) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }
  }

  // ------------
  // Bumper Event
  // ------------

  if (last_state.bumper != new_state.bumper)
  {
    BumperEvent event;

    // Check changes in each bumper state's and raise an event if so
    if ((new_state.bumper ^ last_state.bumper) & CoreSensors::Flags::LeftBumper) {
      event.bumper = BumperEvent::Left;
      if (new_state.bumper & CoreSensors::Flags::LeftBumper) {
        event.state = BumperEvent::Pressed;
      } else {
        event.state = BumperEvent::Released;
      }
      sig_bumper_event.emit(event);
    }

    if ((new_state.bumper ^ last_state.bumper) & CoreSensors::Flags::CenterBumper) {
      event.bumper = BumperEvent::Center;
      if (new_state.bumper & CoreSensors::Flags::CenterBumper) {
        event.state = BumperEvent::Pressed;
      } else {
        event.state = BumperEvent::Released;
      }
      sig_bumper_event.emit(event);
    }

    if ((new_state.bumper ^ last_state.bumper) & CoreSensors::Flags::RightBumper) {
      event.bumper = BumperEvent::Right;
      if (new_state.bumper & CoreSensors::Flags::RightBumper) {
        event.state = BumperEvent::Pressed;
      } else {
        event.state = BumperEvent::Released;
      }
      sig_bumper_event.emit(event);
    }
  }

  // ------------
  // Cliff Event
  // ------------

  if (last_state.cliff != new_state.cliff)
  {
    CliffEvent event;

    // Check changes in each cliff sensor state's and raise an event if so
    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::LeftCliff) {
      event.sensor = CliffEvent::Left;
      if (new_state.cliff & CoreSensors::Flags::LeftCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      event.bottom = cliff_data[event.sensor];
      sig_cliff_event.emit(event);
    }

    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::CenterCliff) {
      event.sensor = CliffEvent::Center;
      if (new_state.cliff & CoreSensors::Flags::CenterCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      event.bottom = cliff_data[event.sensor];
      sig_cliff_event.emit(event);
    }

    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::RightCliff) {
      event.sensor = CliffEvent::Right;
      if (new_state.cliff & CoreSensors::Flags::RightCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      event.bottom = cliff_data[event.sensor];
      sig_cliff_event.emit(event);
    }
  }

  // ------------
  // Wheel Drop Event
  // ------------

  if (last_state.wheel_drop != new_state.wheel_drop)
  {
    WheelEvent event;

    // Check changes in each wheel_drop sensor state's and raise an event if so
    if ((new_state.wheel_drop ^ last_state.wheel_drop) & CoreSensors::Flags::LeftWheel) {
      event.wheel = WheelEvent::Left;
      if (new_state.wheel_drop & CoreSensors::Flags::LeftWheel) {
        event.state = WheelEvent::Dropped;
      } else {
        event.state = WheelEvent::Raised;
      }
      sig_wheel_event.emit(event);
    }

    if ((new_state.wheel_drop ^ last_state.wheel_drop) & CoreSensors::Flags::RightWheel) {
      event.wheel = WheelEvent::Right;
      if (new_state.wheel_drop & CoreSensors::Flags::RightWheel) {
        event.state = WheelEvent::Dropped;
      } else {
        event.state = WheelEvent::Raised;
      }
      sig_wheel_event.emit(event);
    }
  }

  // ------------
  // Power System Event
  // ------------

  if (last_state.charger != new_state.charger)
  {
    Battery battery_new(new_state.battery, new_state.charger);
    Battery battery_last(last_state.battery, last_state.charger);

    if (battery_last.charging_state != battery_new.charging_state)
    {
      PowerEvent event;
      switch (battery_new.charging_state)
      {
        case Battery::Discharging:
          event.event = PowerEvent::Unplugged;
          break;
        case Battery::Charged:
          event.event = PowerEvent::ChargeCompleted;
          break;
        case Battery::Charging:
          if (battery_new.charging_source == Battery::Adapter)
            event.event = PowerEvent::PluggedToAdapter;
          else
            event.event = PowerEvent::PluggedToDockbase;
          break;
      }
      sig_power_event.emit(event);
    }
  }

  if (last_state.battery > new_state.battery)
  {
    Battery battery_new(new_state.battery, new_state.charger);
    Battery battery_last(last_state.battery, last_state.charger);

    if (battery_last.level() != battery_new.level())
    {
      PowerEvent event;
      switch (battery_new.level())
      {
        case Battery::Low:
          event.event = PowerEvent::BatteryLow;
          break;
        case Battery::Dangerous:
          event.event = PowerEvent::BatteryCritical;
          break;
        default:
          break;
      }
      sig_power_event.emit(event);
    }
  }

  last_state = new_state;
}

/**
 * Emit events if something changed in the digital input port.
 * @param new_digital_input New values on digital input port.
 */
void EventManager::update(const uint16_t &new_digital_input)
{
  if (last_digital_input != new_digital_input)
  {
    InputEvent event;

    event.values[0] = new_digital_input&0x0001;
    event.values[1] = new_digital_input&0x0002;
    event.values[2] = new_digital_input&0x0004;
    event.values[3] = new_digital_input&0x0008;

    sig_input_event.emit(event);

    last_digital_input = new_digital_input;
  }
}

} // namespace kobuki
