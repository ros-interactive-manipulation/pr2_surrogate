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


#include "arm_mover.h"

#include <geometry_msgs/PoseStamped.h>

ArmMover::ArmMover( ros::NodeHandle pnh )
{
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>( "joy", 1, boost::bind(&ArmMover::joyCb, this, _1) );

  command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "command", 1 );

  pnh.param<std::string>( "tracked_frame", pose_msg_.header.frame_id, "" );

  pnh.param<int>( "deadman_button", deadman_button_, 0 );

  pnh.param<double>( "update_freq", update_freq_, 0.1 );
}

ArmMover::~ArmMover()
{
}

void ArmMover::joyCb( sensor_msgs::JoyConstPtr joy_msg )
{
  if ( ros::Time::now() - last_update_time_ < ros::Duration(update_freq_) )
  {
    return;
  }

  if ( joy_msg->buttons.size() <= deadman_button_  )
  {
    ROS_ERROR_ONCE("Button index for deadman switch is out of bounds!");
    return;
  }

  if ( joy_msg->buttons.at(deadman_button_) )
  {
    last_update_time_ = ros::Time::now();
    pose_msg_.header.stamp = ros::Time::now();
    pose_msg_.pose.orientation.w = 1;
    command_pub_.publish( pose_msg_ );
  }
}
