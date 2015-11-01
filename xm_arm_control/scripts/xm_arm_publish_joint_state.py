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

# Description: This script publish the joint's position infomation.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


class PublishJointState:
    def __init__(self):
        global joint_dof, joint_position
        global rviz_joint_position
        joint_dof = 6
        joint_position = [0 for i in range(joint_dof)]
        rviz_joint_position = [0 for i in range(joint_dof)]
        rospy.init_node('xm_publish_joint_state', anonymous=False)
        rospy.loginfo("Start to publish joint's states!")
        rospy.Subscriber('xm_arm_controller/state', JointTrajectoryControllerState, self.send_joint_state,
                         queue_size=1000)
        rospy.spin()

    def send_joint_state(self, data):
        global joint_position, joint_dof
        joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
        for i in range(joint_dof):
            joint_position[i] = data.desired.positions[i]
        rospy.logwarn(
            "lift:{0} waist:{1} big_arm:{2} forearm:{3} wrist_pitching:{4} wrist_rotation:{5}".format(
                joint_position[0], joint_position[1], joint_position[2], joint_position[3], joint_position[4],
                joint_position[5]))
        self.joint_state_transform(joint_state_pub)

    def joint_state_transform(self, state_pub):
        global joint_position, joint_dof
        global rviz_joint_position
        joint_name = ['joint_lift',
                      'joint_waist',
                      'joint_big_arm',
                      'joint_forearm',
                      'joint_wrist_pitching',
                      'joint_wrist_rotation']
        rviz_joint_position[0] = -joint_position[0]
        rviz_joint_position[1] = joint_position[1]
        rviz_joint_position[2] = joint_position[2]
        rviz_joint_position[3] = -joint_position[3]
        rviz_joint_position[4] = -joint_position[4]
        rviz_joint_position[5] = joint_position[5]

        rviz_joint_state = JointState()
        rviz_joint_state.header.stamp = rospy.Time.now()
        rviz_joint_state.name = joint_name
        rviz_joint_state.position = [rviz_joint_position[0], rviz_joint_position[1], rviz_joint_position[2],
                                     rviz_joint_position[3], rviz_joint_position[4], rviz_joint_position[5]]
        rviz_joint_state.velocity = [0 for i in range(joint_dof)]
        rviz_joint_state.effort = [0 for i in range(joint_dof)]
        state_pub.publish(rviz_joint_state)


if __name__ == '__main__':
    try:
        publish_joint_state = PublishJointState()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Error!")
