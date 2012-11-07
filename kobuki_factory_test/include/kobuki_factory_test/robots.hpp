/**
 * @file /include/kobuki_factory_test/robots.hpp
 *
 * @brief Evaluated robots list
 *
 * @date October 2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_FACTORY_TEST_ROBOTS_HPP_
#define KOBUKI_FACTORY_TEST_ROBOTS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_factory_test {

/*****************************************************************************
** Types
*****************************************************************************/

typedef long long int int64;

/*****************************************************************************
** Class
*****************************************************************************/

class Robot {

public:
  enum State {
    UNKNOWN = -1,
    OK      =  0,
    WARN    =  1,
    ERROR   =  2
  };

  enum Device {
    V_INFO,
    IR_DOCK_L,
    IR_DOCK_C,
    IR_DOCK_R,
    IMU_DEV,
    BUTTON_0,
    BUTTON_1,
    BUTTON_2,
    BUMPER_L,
    BUMPER_C,
    BUMPER_R,
    W_DROP_L,
    W_DROP_R,
    CLIFF_L,
    CLIFF_C,
    CLIFF_R,
    PWR_JACK,
    PWR_DOCK,
    CHARGING,
    EXT_PWR,
    INPUT,
    OUTPUT,
    LED_1,
    LED_2,
    SOUNDS,
    MOTOR_L,
    MOTOR_R,
    DEVICES_COUNT
  };

  Robot(unsigned int seq_nb) :
    seq_nb(seq_nb),
    state(UNKNOWN),
    device_ok(DEVICES_COUNT),
    device_val(DEVICES_COUNT) {
//    v_info(3),
//    beacon(3),
//    button(3),
//    bumper(3),
//    w_drop(2),
//    cliff(3),
//    power(2),
//    input(4),
//    motor(2),
//    leds(3) {

    for (unsigned int i = 0; i < DEVICES_COUNT; i++) {
      device_ok[i] = false;
      device_val[i] = 0;
    }

//    v_info_ok = false;
//    beacon_ok = false;
//    gyro_ok = false;
//    button_ok = false;
//    bumper_ok = false;
//    w_drop_ok = false;
//    cliff_ok = false;
//    power_ok = false;
//    input_ok = false;
//    output_ok = false;
//    ext_pwr_ok = false;
//    leds_ok = false;
//    sound_ok = false;
//    motor_ok = false;
  };

  ~Robot() { };

  bool all_ok() {
    for (unsigned int i = 0; i < DEVICES_COUNT; i++) {
      if (device_ok[i] == false)
        return false;
    }

    return true;
  };

  bool motors_ok()  { return std::max(device_val[MOTOR_L], device_val[MOTOR_R]) <= 10; }
  bool ir_dock_ok() { return device_ok[IR_DOCK_L] && device_ok[IR_DOCK_C] && device_ok[IR_DOCK_R]; };
  bool buttons_ok() { return device_ok[BUTTON_0]  && device_ok[BUTTON_1]  && device_ok[BUTTON_2]; };
  bool bumpers_ok() { return device_ok[BUMPER_L]  && device_ok[BUMPER_C]  && device_ok[BUMPER_R]; };
  bool w_drop_ok()  { return device_ok[W_DROP_L]  && device_ok[W_DROP_R]; };
  bool cliffs_ok()  { return device_ok[CLIFF_L]   && device_ok[CLIFF_C]   && device_ok[CLIFF_R]; };
  bool pwr_src_ok() { return device_ok[PWR_JACK]  && device_ok[PWR_DOCK]; };

  std::string version_nb() {
    // Version number is stored as 0xhhhhffffssssssss
    //  - first 16 bits for hardware
    //  - next 16 bits for firmware
    //  - remaining 32 bits for software
    char str[64];
    snprintf(str, 64, "%lld/%lld/%lld", (device_val[V_INFO] >> 48),
                                        (device_val[V_INFO] >> 32) & 0xFFFF,
                                        (device_val[V_INFO] & 0xFFFFFFFF));
    return std::string(str);
  };

  unsigned int seq_nb;
  std::string  serial;
  State         state;

  std::string      diagnostics;
  std::vector<bool>  device_ok;
  std::vector<int64> device_val;


//  bool v_info_ok;
//  bool beacon_ok;
//  bool gyro_ok;
//  bool button_ok;
//  bool bumper_ok;
//  bool w_drop_ok;
//  bool cliff_ok;
//  bool power_ok;
//  bool input_ok;
//  bool output_ok;
//  bool ext_pwr_ok;
//  bool leds_ok;
//  bool sound_ok;
//  bool motor_ok;

//  std::vector<int> v_info;
//  std::vector<int> beacon;
//  std::vector<int> button;
//  std::vector<int> bumper;
//  std::vector<int> w_drop;
//  std::vector<int> cliff;
//  std::vector<int> power;
//  std::vector<int> input;
//  std::vector<int> motor;
//  std::vector<int> leds;

};

class RobotList : public std::list<Robot> {

};


// Define a postfix increment operator for Robot::Device
inline Robot::Device operator++(Robot::Device& rd, int)
{
  return rd = (Robot::Device)(rd + 1);
}

}  // namespace kobuki_factory_test

#endif /* KOBUKI_FACTORY_TEST_ROBOTS_HPP_ */
