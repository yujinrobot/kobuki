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
 * @file /kobuki_node/src/nodelet/kobuki_nodelet.cpp
 *
 * @brief Implementation for the ROS Kobuki nodelet
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/thread.hpp>

#include "kobuki_softnode/fake_kobuki_ros.h"


namespace kobuki
{

class SoftKobukiNodelet : public nodelet::Nodelet
{
  public:
    SoftKobukiNodelet(){};
    ~SoftKobukiNodelet()
    {
      NODELET_DEBUG_STREAM("Kobuki : waiting for update thread to finish.");
      update_thread_.join();
    }
    virtual void onInit()
    {
      NODELET_DEBUG_STREAM("Kobuki : initialising nodelet...");
      std::string nodelet_name = this->getName();
      kobuki_.reset(new FakeKobukiRos(nodelet_name));
      // if there are latency issues with callbacks, we might want to move to process callbacks in multiple threads (use MTPrivateNodeHandle)
      if (kobuki_->init(this->getPrivateNodeHandle()))
      {
        update_thread_ = boost::thread(&SoftKobukiNodelet::update,this);
        NODELET_INFO_STREAM("Kobuki : initialised.");
      }
      else
      {
        NODELET_ERROR_STREAM("Kobuki : could not initialise! Please restart.");
      }
    }
  private:
    void update()
    {
      ros::Rate spin_rate(30);
      while (ros::ok() && kobuki_->update())
      {
        spin_rate.sleep();
      }
    }

    boost::shared_ptr<FakeKobukiRos> kobuki_;
    boost::thread update_thread_;
};

} // namespace kobuki

PLUGINLIB_EXPORT_CLASS(kobuki::SoftKobukiNodelet, nodelet::Nodelet);
