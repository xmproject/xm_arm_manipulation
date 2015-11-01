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

# Description: This script can control the arm's position in the rviz.

# Create Date: 2015.11.1

# Authors: myyerrol  


import sys
import tty
import termios
import select
import rospy
from sensor_msgs.msg import JointState


joint_dof = 6
goal_joint_position = [0 for i in range(joint_dof)]
rviz_joint_position = [0 for i in range(joint_dof)]


class KeyboardTeleopRvizArm:
    setting = termios.tcgetattr(sys.stdin)

    def __init__(self):
        rospy.init_node('xm_keyboard_teleop_rviz_arm', anonymous=False)
        joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
        joint_pos_step = rospy.get_param('joint_pos_step', 0.01745)
        print "--------- Teleop arm in the rviz ----------"
        print "Q(R)              W(T)                 E(Y)"
        print "A(F)                                   D(H)"
        print "                  S(G)                     "
        print "-------------------------------------------"
        print "W:Lift-Up S:Waist-Left A:big_arm-Up        "
        print "D:Forearm-Up Q:Wrist_pitching-Up           "
        print "E:Wrist_rotation-Clock--                   "
        print "T:Lift-Down G:Waist-Right F:big_arm-Down   "
        print "H:Forearm-Down R:Wrist_pitching-Down       "
        print "Y:Wrist_rotation-Clock++                   "
        print "Z:Exit!!!                                  "
        try:
            while True:
                keyboard = self.get_key()
                if keyboard == 'w':
                    joint_pos_step = 0.01
                    goal_joint_position[0] += joint_pos_step
                    joint_pos_step = 0.01745
                    if goal_joint_position[0] >= 0.20:
                        goal_joint_position[0] = 0.20
                elif keyboard == "t":
                    joint_pos_step = 0.01
                    goal_joint_position[0] -= joint_pos_step
                    joint_pos_step = 0.01745
                    if goal_joint_position[0] <= -0.20:
                        goal_joint_position[0] = -0.20
                elif keyboard == 's':
                    goal_joint_position[1] += joint_pos_step
                    if goal_joint_position[1] >= 1.047:
                        goal_joint_position[1] = 1.047
                elif keyboard == 'g':
                    goal_joint_position[1] -= joint_pos_step
                    if goal_joint_position[1] <= -1.047:
                        goal_joint_position[1] = -1.047
                elif keyboard == 'a':
                    goal_joint_position[2] += joint_pos_step
                    if goal_joint_position[2] >= 1.396:
                        goal_joint_position[2] = 1.396
                elif keyboard == 'f':
                    goal_joint_position[2] -= joint_pos_step
                    if goal_joint_position[2] <= -1.920:
                        goal_joint_position[2] = -1.920
                elif keyboard == 'd':
                    goal_joint_position[3] += joint_pos_step
                    if goal_joint_position[3] >= 2.234:
                        joint_pos_step[3] = 2.234
                elif keyboard == 'h':
                    goal_joint_position[3] -= joint_pos_step
                    if goal_joint_position[3] <= -2.234:
                        joint_pos_step[3] = -2.234
                elif keyboard == 'q':
                    goal_joint_position[4] += joint_pos_step
                    if goal_joint_position[4] >= 2.182:
                        goal_joint_position[4] = 2.182
                elif keyboard == 'r':
                    goal_joint_position[4] -= joint_pos_step
                    if goal_joint_position[4] <= -2.182:
                        goal_joint_position[4] = -2.182
                elif keyboard == 'e':
                    goal_joint_position[5] += joint_pos_step
                elif keyboard == 'y':
                    goal_joint_position[5] -= joint_pos_step
                elif keyboard == 'z':
                    exit()
                else:
                    pass
                self.send_joint_state(joint_state_pub)
        except Exception as exce:
            print "Error!", exce
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.setting)

    def send_joint_state(self, state_pub):
        joint_name = ['joint_lift',
                      'joint_waist',
                      'joint_big_arm',
                      'joint_forearm',
                      'joint_wrist_pitching',
                      'joint_wrist_rotation']
        rviz_joint_position[0] = -goal_joint_position[0]
        rviz_joint_position[1] = goal_joint_position[1]
        rviz_joint_position[2] = goal_joint_position[2]
        rviz_joint_position[3] = -goal_joint_position[3]
        rviz_joint_position[4] = -goal_joint_position[4]
        rviz_joint_position[5] = goal_joint_position[5]

        rviz_joint_state = JointState()
        rviz_joint_state.header.stamp = rospy.Time.now()
        rviz_joint_state.name = joint_name
        rviz_joint_state.position = [rviz_joint_position[0], rviz_joint_position[1], rviz_joint_position[2],
                                     rviz_joint_position[3], rviz_joint_position[4], rviz_joint_position[5]]
        rviz_joint_state.velocity = [0 for i in range(joint_dof)]
        rviz_joint_state.effort = [0 for i in range(joint_dof)]
        state_pub.publish(rviz_joint_state)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.setting)
        return key


if __name__ == '__main__':
    try:
        KeyboardTeleopRvizArm()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Error!")
