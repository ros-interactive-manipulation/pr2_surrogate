/*********************************************************************
*
*  Copyright (c) 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef ACTION_WRAPPER_H_
#define ACTION_WRAPPER_H_

#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/function.hpp>
#include <ros/node_handle.h>

#include <string>
#include <stdexcept>

#include "exceptions.h"


template <class ActionDataType>
class ActionWrapper
{
 private:
  //! Has the service client been initialized or not
  bool initialized_;
  //! The name of the action
  std::string action_name_;
  //! The remapped name/topic of the action
  std::string remapped_name_;
  //! The node handle to be used when initializing services
  ros::NodeHandle nh_;
  //! The actual action client
  actionlib::SimpleActionClient<ActionDataType> client_;
  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;
 public:
 ActionWrapper(std::string action_name, bool spin_thread) : initialized_(false),
    action_name_(action_name),
    remapped_name_("*name failed to remap!*"),
    nh_(""),
    client_(nh_, action_name, spin_thread) {}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  actionlib::SimpleActionClient<ActionDataType>& client(ros::Duration timeout = ros::Duration(5.0))
  {
    if (!initialized_)
    {
      // aleeper: Added this to aid with remapping debugging.
      remapped_name_ = nh_.resolveName(action_name_, true);
      ros::Duration ping_time = ros::Duration(1.0);
      if (timeout > ros::Duration(0) && ping_time > timeout) ping_time = timeout;
      ros::Time start_time = ros::Time::now();
      while (1)
      {
        if (client_.waitForServer(ping_time)) break;
        if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
        if (!ros::ok()) throw ServiceNotFoundException(action_name_);
        ros::Time current_time = ros::Time::now();
        if (timeout > ros::Duration(0) && current_time - start_time >= timeout)
          throw ServiceNotFoundException(action_name_);
        ROS_INFO_STREAM("Waiting for action client: " << action_name_ << " remapped to " << remapped_name_);
      }
      initialized_ = true;
    }
    return client_;
  }

  bool waitForResult(const ros::Duration &timeout=ros::Duration(0,0))
  {
    ros::Duration ping_time = ros::Duration(5.0);
    if (timeout > ros::Duration(0) && ping_time > timeout) ping_time = timeout;
    ros::Time start_time = ros::Time::now();
    while (1)
    {
      if (client().waitForResult(ping_time)) return true;
      if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
      //we should probably throw something else here
      if (!ros::ok()) throw ServiceNotFoundException(action_name_);
      ros::Time current_time = ros::Time::now();
      if (timeout > ros::Duration(0) && current_time - start_time >= timeout) return false;
      if (!client().isServerConnected()) return false;
      ROS_INFO_STREAM("Waiting for result from action client: " << action_name_ << " remapped to " << remapped_name_);
    }
  }

  bool isInitialized() const {return initialized_;}
};

#endif /* ACTION_WRAPPER_H_ */
