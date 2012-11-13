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

#include <iostream>
#include <fstream>

#include <ros/ros.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_factory_test {

/*****************************************************************************
** Types and constants
*****************************************************************************/

#define AI_MIN  0
#define AI_MAX  1
#define AI_PRE  2
#define AI_INC  3

typedef long long int       int64;
typedef signed short        int16;
typedef unsigned short      uint16;
typedef std::vector<int16>  vint16;

/*****************************************************************************
** Helper functions
*****************************************************************************/

template<class T> std::string to_string(T i) {
    std::stringstream ss;
    std::string s;
    ss << i;
    s = ss.str();

    return ss.str();
}

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
    A_INPUT,
    D_INPUT,
    D_OUTPUT,
    LED_1,
    LED_2,
    SOUNDS,
    MOTOR_L,
    MOTOR_R,
    DEVICES_COUNT
  };

  Robot(unsigned int seq_nb) :
    seq_nb(seq_nb),
    serial("NOT PROVIDED"), // Given when flashing in the robot
    state(UNKNOWN),
    device_ok(DEVICES_COUNT, false),
    device_val(DEVICES_COUNT, std::numeric_limits<int64>::max()),
    imu_data(5), // test 1, diff 1, test 2, diff 2, current value
    analog_in(4, vint16(4)) { // 4 x min/max/prev/inc voltages for analog ports

    for (unsigned int i = 0; i < DEVICES_COUNT; i++) {
      device_ok[i] = false;
      device_val[i] = 0;
    }

    for (unsigned int i = 0; i < analog_in.size(); i++) {
      analog_in[i][AI_MIN] = std::numeric_limits<int16>::max();
      analog_in[i][AI_MAX] = std::numeric_limits<int16>::min();

      analog_in[i][AI_PRE] = -1;
      analog_in[i][AI_INC] =  0;
    }
  };

  ~Robot() { };

  bool all_ok() {
                                              device_ok[EXT_PWR] = true;
    for (unsigned int i = 0; i < DEVICES_COUNT; i++) {
      if (device_ok[i] == false)
        return false;
    }

    return true;
  };

  bool motors_ok()  { return device_ok[MOTOR_L]   && device_ok[MOTOR_R]; };
  bool ir_dock_ok() { return device_ok[IR_DOCK_L] && device_ok[IR_DOCK_C] && device_ok[IR_DOCK_R]; };
  bool buttons_ok() { return device_ok[BUTTON_0]  && device_ok[BUTTON_1]  && device_ok[BUTTON_2]; };
  bool bumpers_ok() { return device_ok[BUMPER_L]  && device_ok[BUMPER_C]  && device_ok[BUMPER_R]; };
  bool w_drop_ok()  { return device_ok[W_DROP_L]  && device_ok[W_DROP_R]; };
  bool cliffs_ok()  { return device_ok[CLIFF_L]   && device_ok[CLIFF_C]   && device_ok[CLIFF_R];  };
  bool pwr_src_ok() { return device_ok[PWR_JACK]  && device_ok[PWR_DOCK]  && device_ok[CHARGING]; };

  std::string version_nb(char separator = '/') {
    // Version number is stored as 0xhhhhffffssssssss
    //  - first 16 bits for hardware
    //  - next 16 bits for firmware
    //  - remaining 32 bits for software
    char str[64];
    snprintf(str, 64, "%lld%c%lld%c%lld", (device_val[V_INFO] >> 48),          separator,
                                          (device_val[V_INFO] >> 32) & 0xFFFF, separator,
                                          (device_val[V_INFO] & 0xFFFFFFFF));
    return std::string(str);
  };

  bool saveToCSVFile(const std::string& path) {
    std::ofstream os;
    os.open(path.c_str(), std::ios::out | std::ios::app);
    if (os.good() == false) {
      ROS_ERROR("Unable to open %s for writing", path.c_str());
      return false;
    }

    if (os.tellp() == 0) {
      // Empty file; write header
//      os << ",SN,DCJ,MOP,DST,VER,FW,HW,DOCK,BRU,S-Brush,,VAC,PSD,,,FIR,,,DIR,,,Forward,,Backward,,PE,,HALL,DOCK,CHR,G-Test,RESULT";
      os << "SN,VER,,,PWR,,,,PSD,,,,BUMP,,,,IR-DOCK,,,,W-DROP,,,MOTORS,,,IMU,,BUTTON,,,LEDS,SNDS,D-IN,D-OUT,A-IN,,,,,,,,,RESULT\n";
      os << ",HW,FW,SW,,JACK,DOCK,CHR,,L,C,R,,L,C,R,,L,C,R,,L,R,,L,R,,difference,F0,F1,F2,,,,,,MIN,MAX,MIN,MAX,MIN,MAX,MIN,MAX,\n";
    }

    os << serial << "," << version_nb(',') << ","
       << pwr_src_ok()  << "," << device_val[PWR_DOCK]  << "," << device_val[PWR_JACK]  << "," << device_val[CHARGING]  << ","
       << cliffs_ok()   << "," << device_val[CLIFF_L]   << "," << device_val[CLIFF_C]   << "," << device_val[CLIFF_R]   << ","
       << buttons_ok()  << "," << device_val[BUMPER_L]  << "," << device_val[BUMPER_C]  << "," << device_val[BUMPER_R]  << ","
       << ir_dock_ok()  << "," << device_val[IR_DOCK_L] << "," << device_val[IR_DOCK_C] << "," << device_val[IR_DOCK_R] << ","
       << w_drop_ok()   << "," << device_val[W_DROP_L]  << "," << device_val[W_DROP_R]  << ","
       << motors_ok()   << "," << device_val[MOTOR_L]   << "," << device_val[MOTOR_R]   << ","
       << device_ok[IMU_DEV]   << "," << imu_data[1] - imu_data[3]  << ","
       << device_ok[BUTTON_0]  << "," << device_ok[BUTTON_1]  << "," << device_ok[BUTTON_2]  << ","
       << (device_ok[LED_1] && device_ok[LED_2]) << "," << device_ok[SOUNDS]     << ","
       << device_ok[D_INPUT]   << "," << device_ok[D_OUTPUT]  << "," << device_ok[A_INPUT]   << ","
       << analog_in[0][AI_MIN] << "," << analog_in[0][AI_MAX] << "," << analog_in[1][AI_MIN] << "," << analog_in[1][AI_MAX] << ","
       << analog_in[2][AI_MIN] << "," << analog_in[2][AI_MAX] << "," << analog_in[3][AI_MIN] << "," << analog_in[3][AI_MAX] << ","

       << all_ok() << std::endl;
//
//    if (!all_ok())
//      os << serial << "," << device_ok[V_INFO] << "," << device_ok[V_INFO] << "," << device_ok[V_INFO] << ","
//         << device_ok[PWR_DOCK]  << "," << device_ok[PWR_JACK]  << "," << device_ok[CHARGING]  << ","
//         << device_ok[CLIFF_L]   << "," << device_ok[CLIFF_C]   << "," << device_ok[CLIFF_R]   << ","
//         << device_ok[BUMPER_L]  << "," << device_ok[BUMPER_C]  << "," << device_ok[BUMPER_R]  << ","
//         << device_ok[IR_DOCK_L] << "," << device_ok[IR_DOCK_C] << "," << device_ok[IR_DOCK_R] << ","
//         << device_ok[W_DROP_L]  << "," << device_ok[W_DROP_R]  << ","
//         << device_ok[MOTOR_L]   << "," << device_ok[MOTOR_R]   << ","
//         << device_ok[IMU_DEV]   << "," << imu_data[1] - imu_data[3] << ","
//         << device_ok[BUTTON_0]  << "," << device_ok[BUTTON_1] << "," << device_ok[BUTTON_2] << ","
//         << device_ok[LED_1]     << "," << device_ok[LED_2]    << "," << device_ok[SOUNDS]   << ","
//         << all_ok() << std::endl;

    os.close();
    return true;

//    EXT_PWR,
  }

  unsigned int seq_nb;
  std::string  serial;
  State         state;

  std::string      diagnostics;
  std::vector<bool>  device_ok;
  std::vector<int64> device_val;

  // Some special data that doesn't fit on device_val
  std::vector<double> imu_data;
  std::vector<vint16> analog_in;
};

class RobotList : public std::list<Robot*> {
public:
  RobotList() {}
  ~RobotList() {
    for (std::list<Robot*>::iterator it = begin(); it != end(); it++) {
      delete *it;
    }
  }
};

// Define a postfix increment operator for Robot::Device
inline Robot::Device operator++(Robot::Device& rd, int)
{
  return rd = (Robot::Device)(rd + 1);
}

}  // namespace kobuki_factory_test

#endif /* KOBUKI_FACTORY_TEST_ROBOTS_HPP_ */
