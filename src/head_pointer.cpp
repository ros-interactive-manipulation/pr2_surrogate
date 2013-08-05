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


#include "head_pointer.h"

HeadPointer::HeadPointer( ros::NodeHandle pnh, std::string action_topic ) :
  point_head_action_client_(action_topic, true)
{
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>( "joy", 1, boost::bind(&HeadPointer::joyCb, this, _1) );

  pnh.param<std::string>( "tracked_frame", point_head_goal_.target.header.frame_id, "oculus" );
  pnh.param<std::string>( "pointing_frame", point_head_goal_.pointing_frame, "head_mount_kinect_rgb_link" );
  pnh.param<int>( "deadman_button", deadman_button_, 0 );

  point_head_goal_.pointing_axis.x = 1;
  point_head_goal_.pointing_axis.y = 0;
  point_head_goal_.pointing_axis.z = 0;

  point_head_goal_.target.point.x = 2;
  point_head_goal_.target.point.y = 0;
  point_head_goal_.target.point.z = 0;

  point_head_goal_.max_velocity = 1.0;

  pnh.param<double>( "update_freq", update_freq_, 0.1 );
}

HeadPointer::~HeadPointer()
{
}

void HeadPointer::joyCb( sensor_msgs::JoyConstPtr joy_msg )
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
    //point_head_goal_.target.header.stamp = ros::Time::now() - ros::Duration(1.0);
    point_head_action_client_.sendGoal( point_head_goal_ );
  }
}
