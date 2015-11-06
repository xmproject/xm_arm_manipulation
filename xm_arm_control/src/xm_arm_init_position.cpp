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

// Description: Initialize arm's position.

// Create Date: 2015.11.1
  
// Authors: myyerrol  


#include <xm_arm_control/xm_arm_init_position.h>


ArmInitTrajectory::ArmInitTrajectory()
{
    trajectory_index_ = 0;
    // Tell the action client that we want to spin a thread by default
    trajectory_client_ = boost::make_shared<TrajectoryClient>(
                        "xm_arm_controller/follow_joint_trajectory", true);
    // Wait for action server to come up
    while(!trajectory_client_->waitForServer(ros::Duration(60)))
        ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
    ROS_INFO_STREAM("Action server is ok!");
}


ArmInitTrajectory::~ArmInitTrajectory()
{
    xm_nh_.shutdown();
}


// Sends the command to start a given trajectory
void ArmInitTrajectory::SendTrajectory(control_msgs::FollowJointTrajectoryGoal
                                         goal)
{
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    trajectory_client_->sendGoal(goal);
    ROS_INFO_STREAM("Send joint trajectory goal to server successfully!");
}


// Generates a simple trajectory with two waypoints, used as an example
// Note that this trajectory contains two waypoints, joined together
// as a single trajectory. Alternatively, each of these waypoints could
// be in its own trajectory - a trajectory can have one or more waypoints
// depending on the desired application.
control_msgs::FollowJointTrajectoryGoal ArmInitTrajectory::ArmTrajectory()
{
    control_msgs::FollowJointTrajectoryGoal goal;
    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("joint_lift");
    goal.trajectory.joint_names.push_back("joint_waist");
    goal.trajectory.joint_names.push_back("joint_big_arm");
    goal.trajectory.joint_names.push_back("joint_forearm");
    goal.trajectory.joint_names.push_back("joint_wrist_pitching");
    goal.trajectory.joint_names.push_back("joint_wrist_rotation");
    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    goal.trajectory.points[trajectory_index_].positions.resize(6);
    goal.trajectory.points[trajectory_index_].positions[0] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[1] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[2] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[3] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[4] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[5] = 0.0;

    goal.trajectory.points[trajectory_index_].velocities.resize(6);
    goal.trajectory.points[trajectory_index_].accelerations.resize(6);
    for (size_t i = 0; i < 6; ++i)
    {
        goal.trajectory.points[trajectory_index_].velocities[i] = 0.0;
        goal.trajectory.points[trajectory_index_].accelerations[i] = 0.0;
    }
    // To be reached 5 second after starting along the trajectory
    goal.trajectory.points[trajectory_index_].time_from_start = ros::Duration(5.0);
    ROS_INFO("Zero trajectory prepare ok!");

    trajectory_index_ += 1;

    goal.trajectory.points[trajectory_index_].positions.resize(6);
    goal.trajectory.points[trajectory_index_].positions[0] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[1] = -1.047;
    goal.trajectory.points[trajectory_index_].positions[2] = 1.047;
    goal.trajectory.points[trajectory_index_].positions[3] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[4] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[5] = 0.0;

    goal.trajectory.points[trajectory_index_].velocities.resize(6);
    goal.trajectory.points[trajectory_index_].accelerations.resize(6);
    for (size_t i = 0; i < 6; ++i)
    {
        goal.trajectory.points[trajectory_index_].velocities[i] = 0.05;
        goal.trajectory.points[trajectory_index_].accelerations[i] = 0.0;
    }
    // To be reached 5 second after starting along the trajectory
    goal.trajectory.points[trajectory_index_].time_from_start = ros::Duration(10);
    ROS_INFO("Initial trajectory prepare ok!");

    return goal;
}


// Returns the current state of the action
actionlib::SimpleClientGoalState ArmInitTrajectory::GetState()
{
    return trajectory_client_->getState();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "xm_arm_action_trajectory");
    ArmInitTrajectory arm_init_trajectory;
    arm_init_trajectory.SendTrajectory(arm_init_trajectory.ArmTrajectory());
    // Wait for trajectory completion
    while(!arm_init_trajectory.GetState().isDone() && ros::ok())
    {
        usleep(50000);
    }
}
