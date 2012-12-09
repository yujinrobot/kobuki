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

namespace kobuki
{

class AutoDocking
{
public:
  AutoDocking(){};
  ~AutoDocking(){};

  bool init() {return true;}
  void auto_dock()
  {
    int i = 0;
    if (ir_left_)
    {
      i = -1;
    }
    else if (ir_right_)
    {
      i = 1;
    }

    target_direction_ = i;
  }

private:
  double magic_;
  int target_direction_;
  bool ir_left_, ir_right_;
  double target_direction;
};

} //namespace kobuki
#endif /* AUTO_DOCKING_HPP_ */
