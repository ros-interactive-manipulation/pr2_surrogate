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


#include "/u/dgossow/wsoculus/src/pr2_surrogate/src/head_pointer.h"

#include <pr2_controllers_msgs/PointHeadGoal.h>
#include "tf/tf.h"

HeadPointer::HeadPointer() :
  point_head_action_client_("head_traj_controller/point_head_action", false)
{
  // TODO Auto-generated constructor stub

}

HeadPointer::~HeadPointer()
{
  // TODO Auto-generated destructor stub
}

bool HeadPointer::pointHeadAction(const geometry_msgs::PointStamped &target, std::string pointing_frame, bool wait_for_result)
{
  pr2_controllers_msgs::PointHeadGoal goal;
  goal.target = target;
  goal.pointing_axis.x = 0;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 1;
  goal.pointing_frame = pointing_frame;
  goal.min_duration = ros::Duration(0.05);
  goal.max_velocity = 1.0;

  point_head_action_client_.client().sendGoal(goal);

  if(wait_for_result)
  {
    point_head_action_client_.client().waitForResult( ros::Duration(3.0) );

    if (point_head_action_client_.client().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_DEBUG_NAMED("manipulation","Successfully moved head.");
      return true;
    }
    else
    {
      ROS_ERROR("Head movement failed or timed out.");
      return false;
    }
  }
  return true;
}
