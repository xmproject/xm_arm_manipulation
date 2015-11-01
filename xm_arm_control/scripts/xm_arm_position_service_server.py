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

# Description: This script bring up the sevice's server to change arm's position 
# to avoid disturbance when recognizing the object and can move arm to the zero
# position.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
import actionlib
from xm_msgs.srv import xm_ArmPosition
from xm_msgs.srv import xm_ArmPositionResponse
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmPositionServer:
    def __init__(self, name):
        # Connect to the xm arm trajectory action server
        self.arm_client = actionlib.SimpleActionClient("xm_arm_controller/follow_joint_trajectory",
                                                       FollowJointTrajectoryAction)
        rospy.init_node('xm_service_server', anonymous=True)
        rospy.Service(name + '/' + 'move_position', xm_ArmPosition, self.arm_position_cb)
        rospy.loginfo("Start service server!")
        rospy.spin()

    def arm_position_cb(self, req):
        global arm_goal
        arm_position = req.position
        state = True
        lift_init_position = 0
        waist_init_angle = 0
        waist_right_max_angle = -1.047
        big_arm_max_up_angle = -1.047
        wrist_pitching_up_angle = 1.047
        wrist_pitching_down_angle = -1.047
        if arm_position == 'init':
            arm_goal = [lift_init_position, waist_right_max_angle, big_arm_max_up_angle, 0, 0, 0]
            print 'arm_position:', arm_goal
        elif arm_position == 'prepare':
            arm_goal = [lift_init_position, waist_init_angle, 0, 0, 0, 0]
            print 'arm_position:', arm_goal
        elif arm_position == 'zero':
            arm_goal = [0, 0, 0, 0, 0, 0]
            print 'arm_position:', arm_goal
        else:
            rospy.logwarn("Please check service client's data!")
            state = False

        if state:
            rospy.loginfo("Preparing for sending arm_goal to the action server")
            rospy.loginfo("Succeeded!")
            self.send_arm_goal(arm_goal)
        else:
            pass
        return xm_ArmPositionResponse(True)

    def send_arm_goal(self, goal):
        # Define joint names
        arm_joint_names = ['joint_lift',
                           'joint_waist',
                           'joint_big_arm',
                           'joint_forearm',
                           'joint_wrist_pitching',
                           'joint_wrist_rotation']

        rospy.loginfo("Waiting for follow_joint_trajectory server")
        self.arm_client.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Connected to follow_joint_trajectory server")
        rospy.sleep(1)
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joint_names
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = goal
        arm_trajectory.points[0].time_from_start = rospy.Duration(4)
        rospy.loginfo("Preparing for moving the arm to goal position!")
        rospy.sleep(1)
        arm_goal_pos = FollowJointTrajectoryGoal()
        arm_goal_pos.trajectory = arm_trajectory
        arm_goal_pos.goal_time_tolerance = rospy.Duration(0)

        self.arm_client.send_goal(arm_goal_pos)
        rospy.loginfo("Send goal to the trajectory server successfully!")
        self.arm_client.wait_for_result(rospy.Duration())


if __name__ == '__main__':
    try:
        arm_position_server = ArmPositionServer('xm_move_arm_position')
    except rospy.ROSInterruptException:
        print "Exit!"
