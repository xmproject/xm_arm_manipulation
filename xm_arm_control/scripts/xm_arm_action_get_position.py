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

# Description: This script get position's data from the action's topic.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
from xm_msgs.msg import xm_MultiJointPos
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def lift_pos_callback(data):
    arm_trajectory = JointTrajectory()
    arm_trajectory.points.append(JointTrajectoryPoint())
    arm_trajectory = data.goal.trajectory
    lift_pos = arm_trajectory.points[0].positions[0]
    joint_position[0] = lift_pos


def arm_pos_callback(data):
    jnt_pos_pub = rospy.Publisher('joint_multi_pos_cmd', xm_MultiJointPos, queue_size=1000)
    rospy.Subscriber('xm_arm_controller/follow_joint_trajectory/goal',
                     FollowJointTrajectoryActionGoal, lift_pos_callback)

    for i in range(1, joint_dof):
        joint_position[i] = data.desired.positions[i]
    rospy.logwarn(
        "[lift]:{0} [waist]:{1} [big_arm]:{2} [forearm]:{3} [wrist_pitching]:{4} [wrist_rotation]:{5}".format(
            joint_position[0], joint_position[1], joint_position[2], joint_position[3], joint_position[4],
            joint_position[5]))
    jnt_pos = xm_MultiJointPos()
    jnt_pos.command = 0x01
    jnt_pos.position = [joint_position[0], joint_position[1], joint_position[2], joint_position[3],
                        joint_position[4],
                        joint_position[5]]
    jnt_pos_pub.publish(jnt_pos)


class ArmActionGetPosition:
    def __init__(self):
        global joint_dof, joint_position
        joint_dof = 6
        joint_position = [0 for i in range(joint_dof)]
        rospy.init_node('xm_get_multi_joint_position', anonymous=False)
        rospy.loginfo("Start to get joint's position!")
        rospy.Subscriber('xm_arm_controller/state', JointTrajectoryControllerState, arm_pos_callback)
        rospy.spin()


if __name__ == '__main__':
    try:
        arm_get_position = ArmActionGetPosition()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Error!")
