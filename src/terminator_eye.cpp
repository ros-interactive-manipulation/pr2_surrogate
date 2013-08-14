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


#include "terminator_eye.h"

#include <geometry_msgs/PoseStamped.h>

TerminatorEye::TerminatorEye( ros::NodeHandle pnh ) :
  projector_on_(false)
{
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>( "joy", 1, boost::bind(&TerminatorEye::joyCb, this, _1) );

  pnh.param<int>( "trigger_button", trigger_button_, 11 );

  ROS_INFO_STREAM("Activate terminator with joystick button " << trigger_button_);
}

TerminatorEye::~TerminatorEye()
{
}

void TerminatorEye::joyCb( sensor_msgs::JoyConstPtr joy_msg )
{
  if ( (unsigned)trigger_button_ >= joy_msg->buttons.size() )
  {
    ROS_ERROR_ONCE("Button index for projector trigger is out of bounds!");
    return;
  }

  bool deadman_pressed = joy_msg->buttons.at(trigger_button_);

  if ( deadman_pressed != projector_on_ )
  {
    projector_on_ = deadman_pressed;
    std::string projector_mode = projector_on_ ? "3" : "2";
    // no dynamic_reconfigure c++ api, so we need to run a system command. :(
    std::string dynparam_str =
        "rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode "
        + projector_mode;
    int exit_code = system(dynparam_str.c_str());
    if ( exit_code != 0 )
    {
      ROS_ERROR_STREAM( "Call to dynamic_reconfigure exited with code " << exit_code );
    }
  }

}
