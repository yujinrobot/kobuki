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
<<<<<<< HEAD
  sig_wheel_drop_event.connect(sigslots_namespace + std::string("/wheel_drop_event"));
  sig_cliff_event.connect(sigslots_namespace + std::string("/cliff_event"));
=======
  sig_cliff_event.connect(sigslots_namespace  + std::string("/cliff_event"));
  sig_wheel_event.connect(sigslots_namespace  + std::string("/wheel_event"));
  sig_input_event.connect(sigslots_namespace  + std::string("/input_event"));
>>>>>>> refs/remotes/origin/fuerte
}

/**
 * Update with incoming data and emit events if necessary.
 * @param new_state  Updated core sensors state
 * @param cliff_data Cliff sensors readings (we include them as an extra information on cliff events)
 */
<<<<<<< HEAD
void EventManager::update(const uint8_t &new_button_state, const uint8_t &new_bumper_state, 
    const uint8_t &new_wheel_drop_state, const uint8_t &new_cliff_state) {

  // ------------
  // Button Event
  // ------------

  if (last_button_state != new_button_state)
=======
void EventManager::update(const CoreSensors::Data &new_state, const std::vector<uint16_t> &cliff_data) {
  if (last_state.buttons != new_state.buttons)
>>>>>>> refs/remotes/origin/fuerte
  {
    // Note that the touch pad means at most one button can be pressed
    // at a time.
    ButtonEvent event;

    // Check changes in each button state's; even if this block of code
    // supports it, two buttons cannot be pressed simultaneously
    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::F0) {
      event.button = ButtonEvent::F0;
      if (new_state.buttons & CoreSensors::Flags::F0) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }

    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::F1) {
      event.button = ButtonEvent::F1;
      if (new_state.buttons & CoreSensors::Flags::F1) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }

    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::F2) {
      event.button = ButtonEvent::F2;
      if (new_state.buttons & CoreSensors::Flags::F2) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }
  }

<<<<<<< HEAD
  // ------------
  // Bumper Event
  // ------------

  if (last_bumper_state != new_bumper_state)
=======
  if (last_state.bumper != new_state.bumper)
>>>>>>> refs/remotes/origin/fuerte
  {
    BumperEvent event;
<<<<<<< HEAD
    if ((new_bumper_state | last_bumper_state) & CoreSensors::Flags::LeftBumper) {
=======

    // Check changes in each bumper state's and raise an event if so
    if ((new_state.bumper ^ last_state.bumper) & CoreSensors::Flags::LeftBumper) {
>>>>>>> refs/remotes/origin/fuerte
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

<<<<<<< HEAD
  // ------------
  // Wheel Drop Event
  // ------------

  if (last_wheel_drop_state != new_wheel_drop_state)
  {
    WheelDropEvent event;
    if ((new_wheel_drop_state | last_wheel_drop_state) & CoreSensors::Flags::LeftWheelDrop) {
      event.wheel_drop = WheelDropEvent::Left;
      if (new_wheel_drop_state & CoreSensors::Flags::LeftWheelDrop) {
        event.state = WheelDropEvent::Dropped;
      } else {
        event.state = WheelDropEvent::Raised;
      }
      sig_wheel_drop_event.emit(event);
    }

    if ((new_wheel_drop_state | last_wheel_drop_state) & CoreSensors::Flags::RightWheelDrop) {
      event.wheel_drop = WheelDropEvent::Right;
      if (new_wheel_drop_state & CoreSensors::Flags::RightWheelDrop) {
        event.state = WheelDropEvent::Dropped;
      } else {
        event.state = WheelDropEvent::Raised;
      }
      sig_wheel_drop_event.emit(event);
    }
    last_wheel_drop_state = new_wheel_drop_state;
  }

  // ------------
  // Cliff Event
  // ------------

  if (last_cliff_state != new_cliff_state)
  {
    CliffEvent event;
    if ((new_cliff_state | last_cliff_state) & CoreSensors::Flags::LeftCliff) {
      event.cliff = CliffEvent::Left;
      if (new_cliff_state & CoreSensors::Flags::LeftCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      sig_cliff_event.emit(event);
    }

    if ((new_cliff_state | last_cliff_state) & CoreSensors::Flags::CentreCliff) {
      event.cliff = CliffEvent::Centre;
      if (new_cliff_state & CoreSensors::Flags::CentreCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      sig_cliff_event.emit(event);
    }

    if ((new_cliff_state | last_cliff_state) & CoreSensors::Flags::RightCliff) {
      event.cliff = CliffEvent::Right;
      if (new_cliff_state & CoreSensors::Flags::RightCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      sig_cliff_event.emit(event);
    }
    last_cliff_state = new_cliff_state;
  }


=======
  if (last_state.cliff != new_state.cliff)
  {
    CliffEvent event;

    // Check changes in each cliff sensor state's and raise an event if so
    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::LeftCliff) {
      event.sensor = CliffEvent::Left;
      if (new_state.cliff & CoreSensors::Flags::LeftCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Safe;
      }
      event.bottom = cliff_data[event.sensor];
      sig_cliff_event.emit(event);
    }

    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::CenterCliff) {
      event.sensor = CliffEvent::Center;
      if (new_state.cliff & CoreSensors::Flags::CenterCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Safe;
      }
      event.bottom = cliff_data[event.sensor];
      sig_cliff_event.emit(event);
    }

    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::RightCliff) {
      event.sensor = CliffEvent::Right;
      if (new_state.cliff & CoreSensors::Flags::RightCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Safe;
      }
      event.bottom = cliff_data[event.sensor];
      sig_cliff_event.emit(event);
    }
  }

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

  last_state = new_state;
>>>>>>> refs/remotes/origin/fuerte
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
