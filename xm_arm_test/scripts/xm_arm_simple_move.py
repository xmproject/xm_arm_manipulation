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

# Description: This script bring up the action's client to test arm's height with
# user's interactional input.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sys import exit
from math import sqrt, atan, sin, asin, cos


class TestArmHeight:
    def __init__(self):
        self.arm_client = actionlib.SimpleActionClient("xm_arm_controller/follow_joint_trajectory",
                                                       FollowJointTrajectoryAction)
        global kinect_to_object_length
        rospy.init_node('arm_height_test', anonymous=False)
        print "--------------- Test Arm Accurate Height ---------------"
        print "Input kinect's x and z message to compute arm's position"
        print "Input objects to lift's height to compute arm's position"
        while True:
            print ">>> ",
            keyboard_cmd = raw_input().split(" ")
            try:
                if keyboard_cmd[0] == "object":
                    if keyboard_cmd[1] == "position":
                        if keyboard_cmd[2] is not None and keyboard_cmd[3] is not None:
                            goal_distance_x = float(keyboard_cmd[2])
                            goal_distance_z = float(keyboard_cmd[3])
                            # Compute trajectory by goal object height
                            # Height is vector, zero point is the lift's center, up is negative and down is positive
                            kinect_view_to_vertical = 1.280
                            kinect_to_object_distance = sqrt(goal_distance_x ** 2 + goal_distance_z ** 2)
                            kinect_to_object_theta = atan(goal_distance_z / goal_distance_x)
                            temp_theta = kinect_view_to_vertical + kinect_to_object_theta
                            kinect_to_object_height = kinect_to_object_distance * cos(temp_theta)
                            kinect_to_object_length = kinect_to_object_distance * sin(temp_theta)
                            kinect_to_lift_center = -0.526
                            lift_to_object_height = kinect_to_lift_center + kinect_to_object_height
                            offset = 0.03
                            lift_to_object_height += offset
                            print "kinect_to_object_height:%lf" % kinect_to_object_height
                            print "kinect_to_object_length:%lf" % kinect_to_object_length
                            print "lift_to_object_height:%lf" % lift_to_object_height
                            big_arm_length = 0.350
                            big_arm_max_up_theta = -1.309
                            big_arm_max_up_height = big_arm_length * sin(big_arm_max_up_theta)
                            big_arm_max_down_theta = 1.396
                            big_arm_max_down_height = big_arm_length * sin(big_arm_max_down_theta)
                            lift_max_up_height = 0.20
                            lift_max_down_height = -0.20
                            forearm_length = 0.295
                            gripper_middle_length = 0.22
                            gripper_end_length = 0.26
                            forearm_to_gripper_middle = forearm_length + gripper_middle_length
                            forearm_to_gripper_end = forearm_length + gripper_end_length
                            big_arm_max_up_length = big_arm_length * cos(big_arm_max_up_theta)
                            big_arm_max_down_length = big_arm_length * cos(big_arm_max_down_theta)
                            # We only use lift_joint, waist_joint, big_arm_joint to move the arm
                            if lift_to_object_height == 0:
                                arm_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                                lift_to_object_length = big_arm_length + forearm_to_gripper_middle
                            elif 0 > lift_to_object_height >= big_arm_max_up_height:
                                big_arm_theta = asin(lift_to_object_height / big_arm_length)
                                arm_goal = [0.0, 0.0, big_arm_theta, 0.0, 0.0, 0.0]
                                lift_to_object_length = big_arm_length * cos(big_arm_theta) + forearm_to_gripper_middle
                            elif 0 < lift_to_object_height <= big_arm_max_down_height:
                                big_arm_theta = asin(lift_to_object_height / big_arm_length)
                                arm_goal = [0.0, 0.0, big_arm_theta, 0.0, 0.0, 0.0]
                                lift_to_object_length = big_arm_length * cos(big_arm_length) + forearm_to_gripper_middle
                            elif lift_to_object_height < big_arm_max_up_height < 0:
                                if (big_arm_max_up_height - lift_to_object_height) <= lift_max_up_height:
                                    lift_up_pos = big_arm_max_up_height - lift_to_object_height
                                    arm_goal = [lift_up_pos, 0.0, big_arm_max_up_theta, 0.0, 0.0, 0.0]
                                    lift_to_object_length = big_arm_max_up_length + forearm_to_gripper_middle
                                else:
                                    rest_up_pos = abs(lift_to_object_height) - (
                                        lift_max_up_height + abs(big_arm_max_up_height))
                                    rospy.logwarn("Achieve the height's the maximum!")
                                    rospy.loginfo("The over up's height is %lf", rest_up_pos)
                                    continue
                            elif lift_to_object_height > big_arm_max_down_height > 0:
                                if (lift_to_object_height - big_arm_max_down_height) <= abs(lift_max_down_height):
                                    lift_down_pos = big_arm_max_down_height - lift_to_object_height
                                    arm_goal = [lift_down_pos, 0.0, big_arm_max_down_theta, 0.0, 0.0, 0.0]
                                    lift_to_object_length = big_arm_max_down_length + forearm_to_gripper_middle
                                else:
                                    rest_down_pos = lift_to_object_height - (
                                        abs(lift_max_down_height) + big_arm_max_down_height)
                                    rospy.logwarn("Achieve the height's the minimum!")
                                    rospy.loginfo("The over down's height is %lf", rest_down_pos)
                                    continue
                            else:
                                rospy.logerr("Error, please check kinect's data!")
                                continue
                            gripper_to_object_length = kinect_to_object_length - lift_to_object_length
                            print "lift_to_object_length:%lf" % lift_to_object_length
                            print "gripper_to_object_length:%lf" % gripper_to_object_length
                            print "arm_position:", arm_goal
                            self.arm_send_goal(arm_goal)

                    elif keyboard_cmd[1] == "height":
                        if keyboard_cmd[2] is not None:
                            lift_to_object_height = float(keyboard_cmd[2])
                            print "lift_to_object_height:%lf" % lift_to_object_height
                            big_arm_length = 0.350
                            big_arm_max_up_theta = -1.309
                            big_arm_max_up_height = big_arm_length * sin(big_arm_max_up_theta)
                            big_arm_max_down_theta = 1.396
                            big_arm_max_down_height = big_arm_length * sin(big_arm_max_down_theta)
                            lift_max_up_height = 0.20
                            lift_max_down_height = -0.20
                            # We only use lift_joint, waist_joint, big_arm_joint to move the arm
                            if lift_to_object_height == 0:
                                arm_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                            elif 0 > lift_to_object_height >= big_arm_max_up_height:
                                big_arm_theta = asin(lift_to_object_height / big_arm_length)
                                arm_goal = [0.0, 0.0, big_arm_theta, 0.0, 0.0, 0.0]
                            elif 0 < lift_to_object_height <= big_arm_max_down_height:
                                big_arm_theta = asin(lift_to_object_height / big_arm_length)
                                arm_goal = [0.0, 0.0, big_arm_theta, 0.0, 0.0, 0.0]
                            elif lift_to_object_height < big_arm_max_up_height < 0:
                                if (big_arm_max_up_height - lift_to_object_height) <= lift_max_up_height:
                                    lift_up_pos = big_arm_max_up_height - lift_to_object_height
                                    arm_goal = [lift_up_pos, 0.0, big_arm_max_up_theta, 0.0, 0.0, 0.0]
                                else:
                                    rest_up_pos = abs(lift_to_object_height) - (
                                        lift_max_up_height + abs(big_arm_max_up_height))
                                    rospy.logwarn("Achieve the height's the maximum!")
                                    rospy.loginfo("The over up's height is %lf", rest_up_pos)
                                    continue
                            elif lift_to_object_height > big_arm_max_down_height > 0:
                                if (lift_to_object_height - big_arm_max_down_height) <= abs(lift_max_down_height):
                                    lift_down_pos = big_arm_max_down_height - lift_to_object_height
                                    arm_goal = [lift_down_pos, 0.0, big_arm_max_down_theta, 0.0, 0.0, 0.0]
                                else:
                                    rest_down_pos = lift_to_object_height - (
                                        abs(lift_max_down_height) + big_arm_max_down_height)
                                    rospy.logwarn("Achieve the height's the minimum!")
                                    rospy.loginfo("The over down's height is %lf", rest_down_pos)
                                    continue
                            else:
                                rospy.logerr("Error, please check kinect's data!")
                                continue
                            print "arm_position:", arm_goal
                            self.arm_send_goal(arm_goal)
                    else:
                        rospy.logwarn("Please input [object position x z] or [object height h]")
                        continue
                elif keyboard_cmd[0] == "zero":
                    arm_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    rospy.loginfo("Move to the all joint's zero position!")
                    self.arm_send_goal(arm_zero)
                elif keyboard_cmd[0] == "exit" or "e":
                    exit()
            except Exception as exce:
                print "Error!", exce

    def arm_send_goal(self, arm_goal):
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
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].time_from_start = rospy.Duration(10)
        rospy.loginfo("Preparing for moving the arm to goal position!")
        rospy.sleep(1)
        arm_goal_pos = FollowJointTrajectoryGoal()
        arm_goal_pos.trajectory = arm_trajectory

        arm_goal_pos.goal_time_tolerance = rospy.Duration(0)
        self.arm_client.send_goal(arm_goal_pos)
        rospy.loginfo("Send goal to the trajectory server successfully!")
        self.arm_client.wait_for_result(rospy.Duration(0))


if __name__ == '__main__':
    try:
        arm_move = TestArmHeight()
    except KeyboardInterrupt:
        print "\nExit!"
