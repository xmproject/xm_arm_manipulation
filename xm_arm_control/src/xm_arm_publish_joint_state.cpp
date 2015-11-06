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

// Description: Publish arm's states from the action's topic.

// Create Date: 2015.11.1
  
// Authors: myyerrol  


#include <xm_arm_control/xm_arm_publish_joint_state.h>


ArmPublishJointState::ArmPublishJointState()
{
    joint_dof_ = 6;
    joint_position_.resize(joint_dof_, 0);
    joint_state_pub_ = xm_nh_.advertise<sensor_msgs::JointState>("joint_states", 1000);
    joint_pos_sub_ = xm_nh_.subscribe("xm_arm_controller/state", 1000,
                                     &ArmPublishJointState::JointPosCallback, this);
    ros::spin();
}


ArmPublishJointState::~ArmPublishJointState()
{
    xm_nh_.shutdown();
}


void ArmPublishJointState::JointPosCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    for(int i = 0; i < joint_dof_; i++)
        joint_position_[i] = msg->desired.positions[i];

    ROS_INFO("joint_lift[%lf] joint_waist[%lf] joint_big_arm[%lf] joint_forearm[%lf]"
             "joint_wrist_pitching[%lf] joint_wrist_rotation[%lf]",
             joint_position_[0], joint_position_[1], joint_position_[2],
             joint_position_[3], joint_position_[4], joint_position_[5]);

    sensor_msgs::JointState xm_joint_state;
    xm_joint_state.header.stamp = ros::Time::now();
    xm_joint_state.name.resize(joint_dof_);
    xm_joint_state.name[0] = "joint_lift";
    xm_joint_state.name[1] = "joint_waist";
    xm_joint_state.name[2] = "joint_big_arm";
    xm_joint_state.name[3] = "joint_forearm";
    xm_joint_state.name[4] = "joint_wrist_pitching";
    xm_joint_state.name[5] = "joint_wrist_rotation";
    xm_joint_state.position.resize(joint_dof_);
    for(int i = 0; i < joint_dof_; i++)
        xm_joint_state.position[i] = joint_position_[i];
    xm_joint_state.velocity.resize(joint_dof_);
    for(int i = 0; i < joint_dof_; i++)
        xm_joint_state.velocity[i] = 0;
    xm_joint_state.effort.resize(joint_dof_);
    for(int i = 0; i < joint_dof_; i++)
        xm_joint_state.effort[i] = 0;

    joint_state_pub_.publish(xm_joint_state);
    ROS_INFO("Send xm joint state successfully!");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "xm_joint_states");

    ArmPublishJointState();

    return 0;
}
