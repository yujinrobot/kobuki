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
** Includes
*****************************************************************************/

#include "kobuki_auto_docking/auto_docking.hpp"

namespace kobuki
{

AutoDocking::AutoDocking()
{;}

AutoDocking::~AutoDocking()
{;}

bool AutoDocking::init() 
{
  return true;
}

void AutoDocking::enable(std::string msg) 
{
  return;
}

void AutoDocking::disable(std::string msg) 
{
  return;
}

void AutoDocking::auto_dock()
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

} //namespace kobuki
