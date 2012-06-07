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
    Centre,
    Right
  } bumper;
};

struct WheelDropEvent {
  enum State {
    Raised,
    Dropped
  } state;
  enum WheelDrop {
    Left,
    Right
  } wheel_drop;
};

struct CliffEvent {
  enum State {
    Floor,
    Cliff
  } state;
  enum Cliff {
    Left,
    Centre,
    Right
  } cliff;
};

/*****************************************************************************
** Interfaces
*****************************************************************************/

class EventManager {
public:
  EventManager() : 
    last_button_state(0), last_bumper_state(0), last_wheel_drop_state(0), last_cliff_state(0) 
  {}

  void init(const std::string &sigslots_namespace);
  void update(const uint8_t &new_button_state, const uint8_t &new_bumper_state, 
    const uint8_t &new_wheel_drop_state, const uint8_t &new_cliff_state);

private:
  uint8_t last_button_state, last_bumper_state, last_wheel_drop_state, last_cliff_state;
  ecl::Signal<const ButtonEvent&> sig_button_event;
  ecl::Signal<const BumperEvent&> sig_bumper_event;
  ecl::Signal<const WheelDropEvent&> sig_wheel_drop_event;
  ecl::Signal<const CliffEvent&> sig_cliff_event;
};


} // namespace kobuki

#endif /* KOBUKI_BUTTON_EVENT_HPP_ */
