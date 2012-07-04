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
 * @file /kobuki_driver/include/kobuki_driver/events/event_manager.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 14/04/2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_BUTTON_EVENT_HPP_
#define KOBUKI_BUTTON_EVENT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdint.h>
#include <vector>
#include <ecl/sigslots.hpp>

#include "packets/core_sensors.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Event Structures
*****************************************************************************/

struct ButtonEvent {
  enum State {
    Released,
    Pressed
  } state;
  enum Button {
    F0,
    F1,
    F2
  } button;
};

struct BumperEvent {
  enum State {
    Released,
    Pressed
  } state;
  enum Bumper {
    Left,
    Center,
    Right
  } bumper;
};

struct CliffEvent {
  enum State {
    Floor,
    Cliff
  } state;
  enum Sensor {
    Left,
    Center,
    Right
  } sensor;
  uint16_t bottom;
};

struct WheelEvent {
  enum State {
    Raised,
    Dropped
  } state;
  enum Wheel {
    Left,
    Right
  } wheel;
};

struct PowerEvent {
  enum Event {
    Unplugged         = 0,
    PluggedToAdapter  = 1,
    PluggedToDockbase = 2,
    ChargeCompleted   = 3,
    BatteryLow        = 4,
    BatteryCritical   = 5
  } event;
};

struct InputEvent {
  bool values[4]; /**< Digital on or off for pins 0-3 respectively. **/
};

/*****************************************************************************
** Interfaces
*****************************************************************************/

class EventManager {
public:
  EventManager() {
    last_state.buttons    = 0;
    last_state.bumper     = 0;
    last_state.cliff      = 0;
    last_state.wheel_drop = 0;
    last_state.charger    = 0;
    last_state.battery    = 0;
    last_digital_input    = 0;
  }

  void init(const std::string &sigslots_namespace);
  void update(const CoreSensors::Data &new_state, const std::vector<uint16_t> &cliff_data);
  void update(const uint16_t &digital_input);

private:
  CoreSensors::Data last_state;
  uint16_t  last_digital_input;

  ecl::Signal<const ButtonEvent&> sig_button_event;
  ecl::Signal<const BumperEvent&> sig_bumper_event;
  ecl::Signal<const CliffEvent&>  sig_cliff_event;
  ecl::Signal<const WheelEvent&>  sig_wheel_event;
  ecl::Signal<const PowerEvent&>  sig_power_event;
  ecl::Signal<const InputEvent&>  sig_input_event;
};


} // namespace kobuki

#endif /* KOBUKI_BUTTON_EVENT_HPP_ */
