#!/usr/bin/env python
'''
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
'''

# Description: This script use trajectory's action to control the arm's position.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmSimpleTrajectory:
    def __init__(self):
        rospy.init_node('arm_simple_trajectory')
        
        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)
        
        # Which joints define the arm?
        arm_joints = ['joint_lift',
                      'joint_waist',
                      'joint_big_arm', 
                      'joint_forearm',
                      'joint_wrist_pitching',
                      'joint_wrist_rotation']
        
        if reset:
            # Set the arm back to the resting position
            arm_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        else:
            # Set a goal configuration for the arm
            arm_goal = [1.0, 1.0, 1.0, 1.0, 0.0, 0.0]
    
        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for xm arm trajectory controller...')
        
        arm_client = actionlib.SimpleActionClient('xm_arm_controller/follow_joint_trajectory',
                                                  FollowJointTrajectoryAction)
        
        arm_client.wait_for_server()
        
        rospy.loginfo('...connected.')
        rospy.sleep(1)
    
        # Create a single-point arm trajectory with the arm_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3)
    
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')
        rospy.sleep(1)
        
        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()
        
        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory
        
        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0)
    
        # Send the goal to the action server
        arm_client.send_goal(arm_goal)

        rospy.loginfo('...done')
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        ArmSimpleTrajectory()
    except rospy.ROSInterruptException:
        pass
