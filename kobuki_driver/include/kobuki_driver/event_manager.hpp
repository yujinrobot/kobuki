/**
 * @file include/kobuki_driver/event_manager.hpp
 *
 * @brief The event manager - sigslot interface.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/master/kobuki_driver/LICENSE
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
    Button0,
    Button1,
    Button2
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

struct RobotEvent {
  enum State {
    Offline,
    Online,
    Unknown  // at startup
  } state;
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
    last_robot_state      = RobotEvent::Unknown;
  }

  void init(const std::string &sigslots_namespace);
  void update(const CoreSensors::Data &new_state, const std::vector<uint16_t> &cliff_data);
  void update(const uint16_t &digital_input);
  void update(bool is_plugged, bool is_alive);

private:
  CoreSensors::Data last_state;
  uint16_t          last_digital_input;
  RobotEvent::State last_robot_state;

  ecl::Signal<const ButtonEvent&> sig_button_event;
  ecl::Signal<const BumperEvent&> sig_bumper_event;
  ecl::Signal<const CliffEvent&>  sig_cliff_event;
  ecl::Signal<const WheelEvent&>  sig_wheel_event;
  ecl::Signal<const PowerEvent&>  sig_power_event;
  ecl::Signal<const InputEvent&>  sig_input_event;
  ecl::Signal<const RobotEvent&>  sig_robot_event;
};


} // namespace kobuki

#endif /* KOBUKI_BUTTON_EVENT_HPP_ */
