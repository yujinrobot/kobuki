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
#include "../../include/kobuki_driver/modules/core_sensors.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

void EventManager::init ( const std::string &sigslots_namespace ) {
  sig_button_event.connect(sigslots_namespace + std::string("/button_event"));
}

/**
 * Update with incoming data and emit events if necessary.
 * @param new_button_state
 */
void EventManager::update(const uint8_t &new_button_state) {
  if (last_button_state != new_button_state)
  {
    // Note that the touch pad means at most one button can be pressed
    // at a time.
    ButtonEvent event;
    // Check changes in each button state's; event if this block of code
    // supports it, two buttons cannot be pressed simultaneously
    if ((new_button_state | last_button_state) & CoreSensors::Flags::F0) {
      event.button = ButtonEvent::F0;
      if (new_button_state & CoreSensors::Flags::F0) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }

    if ((new_button_state | last_button_state) & CoreSensors::Flags::F1) {
      event.button = ButtonEvent::F1;
      if (new_button_state & CoreSensors::Flags::F1) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }

    if ((new_button_state | last_button_state) & CoreSensors::Flags::F2) {
      event.button = ButtonEvent::F2;
      if (new_button_state & CoreSensors::Flags::F2) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
      sig_button_event.emit(event);
    }
    last_button_state = new_button_state;
  }
}


} // namespace kobuki
