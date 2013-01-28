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
 * @file /kobuki_controller_tutorial/src/nodelet.cpp
 *
 * @brief Nodelet implementation of the BumpBlinkController
 *
 * @author Marcus Liebhardt, Yujin Robot
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "kobuki_controller_tutorial/bump_blink_controller.hpp"


namespace kobuki
{

/**
 * @brief Nodelet-wrapper of the BumpBlinkController class
 */
class BumpBlinkControllerNodelet : public nodelet::Nodelet
{
public:
  BumpBlinkControllerNodelet(){};
  ~BumpBlinkControllerNodelet(){}

  /**
   * @brief Initialise the nodelet
   *
   * This function is called, when the nodelet manager loads the nodelet.
   */
  virtual void onInit()
  {
    ros::NodeHandle nh = this->getPrivateNodeHandle();

    // resolve node(let) name
    std::string name = nh.getUnresolvedNamespace();
    int pos = name.find_last_of('/');
    name = name.substr(pos + 1);

    NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
    controller_.reset(new BumpBlinkController(nh, name));

    // Initialises the controller
    if (controller_->init())
    {
      NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
    }
    else
    {
      NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
    }
  }
private:
  boost::shared_ptr<BumpBlinkController> controller_;
};

} // namespace kobuki

PLUGINLIB_EXPORT_CLASS(kobuki::BumpBlinkControllerNodelet,
                       nodelet::Nodelet);
// %EndTag(FULLTEXT)%
