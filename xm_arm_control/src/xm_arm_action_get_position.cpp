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

// Description: Get joint's position from the action's topic.

// Create Date: 2015.11.1
  
// Authors: myyerrol  


#include <xm_arm_control/xm_arm_action_get_position.h>


ArmActionGetPosition::ArmActionGetPosition()
{
    joint_num_ = 6;
    trajectory_num_ = 2;
    wrist_switch_flag_ = 0;
    joint_position_.resize(joint_num_, 0);
    joint_position_current_.resize(joint_num_, 0);
    trajectory_duration_.resize(trajectory_num_, 0);
    joint_pos_pub_ = xm_nh_.advertise<xm_msgs::xm_JointPos>("joint_pos_cmd", 1000);
    joint_pos_sub_ = xm_nh_.subscribe("xm_arm_controller/state", 1000,
                                      &ArmActionGetPosition::JointPosCallback, this);
    ros::spin();
}


ArmActionGetPosition::~ArmActionGetPosition()
{
    xm_nh_.shutdown();
}


void ArmActionGetPosition::JointPosCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
{
    double way_points = 50;
    double waist_pos_step;
    double wrist_pos_step;
    double judge_value;
    ros::Rate rate(10);

    for(int i = 0; i < joint_num_; i++)
        joint_position_[i] = msg->desired.positions[i];
    trajectory_duration_[0] = msg->desired.time_from_start.toSec();

    waist_pos_step = (joint_position_[1] - joint_position_current_[1]) / way_points;
    wrist_pos_step = (joint_position_[4] - joint_position_current_[4]) / way_points;
    ROS_INFO("waist_pos_step:%lf", waist_pos_step);
    ROS_INFO("wrist_pos_step:%lf", wrist_pos_step);

    ROS_INFO("lift[%lf] waist[%lf] big_arm[%lf] forearm[%lf] wrist_pitching[%lf] wrist_rotation[%lf]",
             joint_position_[0], joint_position_[1], joint_position_[2],
             joint_position_[3], joint_position_[4], joint_position_[5]);

    //waist_switch_flag = 0 use user's interpolate
    //waist_switch_flag = 1 use controller's interpolate
    xm_msgs::xm_JointPos xm_jnt_pos;
    if(joint_position_[2] != joint_position_current_[2])
    {
        xm_jnt_pos.command = 0x01;
        xm_jnt_pos.joint = 2;
        xm_jnt_pos.position = joint_position_[2];
        joint_pos_pub_.publish(xm_jnt_pos);
        joint_position_current_[2] = joint_position_[2];
        waist_switch_flag_ = 1;
        wrist_switch_flag_ = 1;
    }
    else if(joint_position_[0] != joint_position_current_[0])
    {
        xm_jnt_pos.command  = 0x01;
        xm_jnt_pos.joint    = 0;
        xm_jnt_pos.position = joint_position_[0];
        joint_pos_pub_.publish(xm_jnt_pos);
        joint_position_current_[0] = joint_position_[0];
        waist_switch_flag_ = 1;
        wrist_switch_flag_ = 1;
    }
    else if(waist_switch_flag_ == 1)
    {
        ROS_WARN("waist");
        if(fabs(joint_position_[1]) > fabs(joint_position_current_[1]))
        {
            judge_value = 1e-10;
            while(fabs(joint_position_[1]) - fabs(joint_position_current_[1]) > judge_value)
            {
                joint_position_current_[1] += waist_pos_step;
                xm_jnt_pos.command = 0x01;
                xm_jnt_pos.joint = 1;
                xm_jnt_pos.position = joint_position_current_[1];
                joint_pos_pub_.publish(xm_jnt_pos);
                rate.sleep();
                ROS_INFO("waist_position_goal:%lf", joint_position_[1]);
                ROS_INFO("waist_position_current:%lf", joint_position_current_[1]);
            }
        }
        else if(fabs(joint_position_[1]) < fabs(joint_position_current_[1]))
        {
            judge_value = -1e-10;
            while(fabs(joint_position_[1]) - fabs(joint_position_current_[1]) < judge_value)
            {
                joint_position_current_[1] += waist_pos_step;
                xm_jnt_pos.command = 0x01;
                xm_jnt_pos.joint = 1;
                xm_jnt_pos.position = joint_position_current_[1];
                joint_pos_pub_.publish(xm_jnt_pos);
                rate.sleep();
                ROS_INFO("waist_position_goal:%lf", joint_position_[1]);
                ROS_INFO("waist_position_current:%lf", joint_position_current_[1]);
            }
        }
        waist_switch_flag_ = 2;
    }
    else if(wrist_switch_flag_ == 1)
    {
        ROS_WARN("wrist1");
        if(fabs(joint_position_[4]) > fabs(joint_position_current_[4]))
        {
             judge_value = 1e-10;
             while(fabs(joint_position_[4]) - fabs(joint_position_current_[4]) > judge_value)
             {
                 joint_position_current_[4] += wrist_pos_step;
                 xm_jnt_pos.command = 0x01;
                 xm_jnt_pos.joint = 4;
                 xm_jnt_pos.position = joint_position_current_[4];
                 joint_pos_pub_.publish(xm_jnt_pos);
                 rate.sleep();
                 ROS_INFO("wrist_position_goal:%lf", joint_position_[4]);
                 ROS_INFO("wrist_position_current:%lf", joint_position_current_[4]);
             }
        }
        else if(fabs(joint_position_[4]) < fabs(joint_position_current_[4]))
        {
            judge_value = -1e-10;
            while(fabs(joint_position_[4]) - fabs(joint_position_current_[4]) < judge_value)
            {
                joint_position_current_[4] += wrist_pos_step;
                xm_jnt_pos.command = 0x01;
                xm_jnt_pos.joint = 4;
                xm_jnt_pos.position = joint_position_current_[4];
                joint_pos_pub_.publish(xm_jnt_pos);
                rate.sleep();
                ROS_INFO("wrist_position_goal:%lf", joint_position_[4]);
                ROS_INFO("wrist_position_current:%lf", joint_position_current_[4]);
            }
        }
        wrist_switch_flag_ = 2;
    }
    else if(joint_position_[1] != joint_position_current_[1])
    {
        xm_jnt_pos.command = 0x01;
        xm_jnt_pos.joint = 1;
        xm_jnt_pos.position = joint_position_[1];
        joint_pos_pub_.publish(xm_jnt_pos);
        joint_position_current_[1] = joint_position_[1];
        wrist_switch_flag_ = 3;
    }
    else if(wrist_switch_flag_ == 3)
    {
        ROS_WARN("wrist2");
        if(fabs(joint_position_[4]) > fabs(joint_position_current_[4]))
        {
             judge_value = 1e-10;
             while(fabs(joint_position_[4]) - fabs(joint_position_current_[4]) >
                   judge_value)
             {
                 joint_position_current_[4] += wrist_pos_step;
                 xm_jnt_pos.command = 0x01;
                 xm_jnt_pos.joint = 4;
                 xm_jnt_pos.position = joint_position_current_[4];
                 joint_pos_pub_.publish(xm_jnt_pos);
                 rate.sleep();
                 ROS_INFO("wrist_position_goal:%lf", joint_position_[4]);
                 ROS_INFO("wrist_position_current:%lf", joint_position_current_[4]);
             }
        }
        else if(fabs(joint_position_[4]) < fabs(joint_position_current_[4]))
        {
            judge_value = -1e-10;
            while(fabs(joint_position_[4]) - fabs(joint_position_current_[4]) < judge_value)
            {
                joint_position_current_[4] += wrist_pos_step;
                xm_jnt_pos.command = 0x01;
                xm_jnt_pos.joint = 4;
                xm_jnt_pos.position = joint_position_current_[4];
                joint_pos_pub_.publish(xm_jnt_pos);
                rate.sleep();
                ROS_INFO("wrist_position_goal:%lf", joint_position_[4]);
                ROS_INFO("wrist_position_current:%lf", joint_position_current_[4]);
            }
        }
        wrist_switch_flag_ = 4;
    }
    else
        return ;
    ROS_ERROR("waist_goal:%lf", joint_position_[1]);
    ROS_ERROR("waist_curr:%lf", joint_position_current_[1]);
    ROS_ERROR("wrist_goal:%lf", joint_position_[4]);
    ROS_ERROR("wrist_curr:%lf", joint_position_current_[4]);
    ROS_ERROR("waist_flag:%d", waist_switch_flag_);
    ROS_ERROR("wrist_flag:%d", wrist_switch_flag_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "xm_get_joint_position");

    ArmActionGetPosition();

    return 0;
}
