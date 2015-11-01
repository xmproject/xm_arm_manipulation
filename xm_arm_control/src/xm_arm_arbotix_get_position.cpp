/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Created for the XM Robot Project: http://www.github/xmproject
 *  Copyright (c) 2015 The XM Robot Team. All rights reserved
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
 *   * Neither the name of XM Robot Project nor the names of its
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

// Description: Get joint's position from the action's topic by using the arbotix
// controller.

// Create Data: 2015.11.1
  
// Authors: myyerrol  


#include <xm_arm_control/xm_arm_arbotix_get_position.h>


ArmArbotixGetPosition::ArmArbotixGetPosition()
{
    joint_dof_ = 6;
    joint_position_.resize(joint_dof_, 0);
    joint_pos_pub_ = xm_nh_.advertise<xm_msgs::xm_JointPos>("joint_pos_cmd", 1000);
    joint_pos_sub_ = xm_nh_.subscribe("joint_states", 1000,
                                      &ArmArbotixGetPosition::JointPosCallback, this);
    ros::spin();
}


ArmArbotixGetPosition::~ArmArbotixGetPosition()
{
    xm_nh_.shutdown();
}


void ArmArbotixGetPosition::JointPosCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    ROS_INFO("joint_lift[%lf] joint_waist[%lf] joint_big_arm[%lf] joint_forearm[%lf]",
             msg->position[0], msg->position[3], msg->position[5], msg->position[1]);

    joint_position_[0] = msg->position[0];
    joint_position_[1] = msg->position[3];
    joint_position_[2] = msg->position[5];
    joint_position_[3] = msg->position[1];

    for (int i = 0; i < joint_dof_ - 2; i++)
    {
        xm_joint_pos_.command = 0x01;
        xm_joint_pos_.joint = i;
        xm_joint_pos_.position = joint_position_[i];
        joint_pos_pub_.publish(xm_joint_pos_);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "xm_arbotix_get_joint_position");
    ROS_INFO_STREAM("Wait for the /joint_states topic!");

    ArmArbotixGetPosition();

    return 0;
}
