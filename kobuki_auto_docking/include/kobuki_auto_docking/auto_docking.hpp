/**
 * @file /auto_docking/include/auto_docking/auto_docking.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Nov 30, 2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef AUTO_DOCKING_HPP_
#define AUTO_DOCKING_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ecl/geometry/pose2d.hpp>
#include <string>

namespace kobuki
{

class AutoDocking
{
public:
  AutoDocking();
  ~AutoDocking();

  bool init();
  void enable(std::string msg);
  void disable(std::string msg);
  void auto_dock();

private:
  double magic_;
  int target_direction_;
  bool ir_left_, ir_right_;
  double target_direction;
};

} //namespace kobuki
#endif /* AUTO_DOCKING_HPP_ */
