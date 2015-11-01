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

# Description: This script bring up the action's server to control the joint of 
# wrist's pitching.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
import actionlib
from xm_msgs.msg import xm_WristPositionAction, xm_WristPositionResult
from xm_msgs.msg import xm_JointPos


class WristActionServer(object):
    result = xm_WristPositionResult()

    def __init__(self, name):
        global wrist_pos_current
        wrist_pos_current = 0
        self.wrist_action_name = name
        self.wrist_action_server = actionlib.SimpleActionServer(self.wrist_action_name, xm_WristPositionAction,
                                                                execute_cb=self.execute_cb, auto_start=False)
        self.wrist_action_server.start()
        self.wrist_pos_pub = rospy.Publisher('/joint_pos_cmd', xm_JointPos, queue_size=1000)
        rospy.loginfo("Start action server!")

    def execute_cb(self, goal):
        rate = rospy.Rate(20)
        success = True
        rospy.loginfo("Receiving wrist's angle %lf", goal.angle)

        if self.wrist_action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.wrist_action_name)
            self.wrist_action_server.set_preempted()
            success = False
        rate.sleep()

        self.send_wrist_goal_single(goal.angle)

        if success:
            self.result.complete = True
            rospy.loginfo("Succeeded!")
            self.wrist_action_server.set_succeeded(self.result)

    def send_wrist_goal_single(self, wrist_goal):
        global judge_value, wrist_pos_current
        wrist_pos = xm_JointPos()
        way_points = 50
        wrist_pos_step = (wrist_goal - wrist_pos_current) / way_points
        rospy.loginfo("wrist_step:%lf", wrist_pos_step)
        rospy.loginfo("Publish wrist's pitching angle to the topic!")
        if (wrist_goal >= 0 and wrist_pos_current >= 0) or (wrist_goal <= 0 and wrist_pos_current <= 0):
            if abs(wrist_goal) > abs(wrist_pos_current):
                judge_value = 1E-10
                while(abs(wrist_goal) - abs(wrist_pos_current) >= judge_value) and not rospy.is_shutdown():
                    wrist_pos_current += wrist_pos_step
                    wrist_pos.command = 0x01
                    wrist_pos.joint = 4
                    wrist_pos.position = wrist_pos_current
                    self.wrist_pos_pub.publish(wrist_pos)
                    rospy.sleep(0.1)
                    rospy.loginfo("wrist_goal_pos:%f", wrist_goal)
                    rospy.loginfo("wrist_current_pos:%f", wrist_pos_current)
                    rospy.loginfo("---")
            elif abs(wrist_goal) < abs(wrist_pos_current):
                judge_value = -1E-10
                while(abs(wrist_goal) - abs(wrist_pos_current) <= judge_value) and not rospy.is_shutdown():
                    wrist_pos_current += wrist_pos_step
                    wrist_pos.command = 0x01
                    wrist_pos.joint = 4
                    wrist_pos.position = wrist_pos_current
                    self.wrist_pos_pub.publish(wrist_pos)
                    rospy.sleep(0.1)
                    rospy.loginfo("wrist_goal_pos:%f", wrist_goal)
                    rospy.loginfo("wrist_current_pos:%f", wrist_pos_current)
                    rospy.loginfo("---")
            wrist_pos_current = wrist_goal
        elif wrist_goal > 0 > wrist_pos_current:
            judge_value = -1E-10
            wrist_pos_step = (0 - wrist_pos_current) / way_points
            while(0 - abs(wrist_pos_current) <= judge_value) and not rospy.is_shutdown():
                wrist_pos_current += wrist_pos_step
                wrist_pos.command = 0x01
                wrist_pos.joint = 4
                wrist_pos.position = wrist_pos_current
                self.wrist_pos_pub.publish(wrist_pos)
                rospy.sleep(0.1)
                rospy.loginfo("wrist_goal_pos:%f", wrist_goal)
                rospy.loginfo("wrist_current_pos:%f", wrist_pos_current)
                rospy.loginfo("---")
            wrist_pos_current = 0
            judge_value = 1E-10
            wrist_pos_step = (wrist_goal - 0) / way_points
            while(wrist_goal - wrist_pos_current >= judge_value) and not rospy.is_shutdown():
                wrist_pos_current += wrist_pos_step
                wrist_pos.command = 0x01
                wrist_pos.joint = 4
                wrist_pos.position = wrist_pos_current
                self.wrist_pos_pub.publish(wrist_pos)
                rospy.sleep(0.1)
                rospy.loginfo("wrist_goal_pos:%f", wrist_goal)
                rospy.loginfo("wrist_current_pos:%f", wrist_pos_current)
                rospy.loginfo("---")
            wrist_pos_current = wrist_goal
        elif wrist_goal < 0 < wrist_pos_current:
            judge_value = -1E-10
            wrist_pos_step = (0 - wrist_pos_current) / way_points
            while (0 - wrist_pos_current <= judge_value) and not rospy.is_shutdown():
                wrist_pos_current += wrist_pos_step
                wrist_pos.command = 0x01
                wrist_pos.joint = 4
                wrist_pos.position = wrist_pos_current
                self.wrist_pos_pub.publish(wrist_pos)
                rospy.sleep(0.1)
                rospy.loginfo("wrist_goal_pos:%f", wrist_goal)
                rospy.loginfo("wrist_current_pos:%f", wrist_pos_current)
                rospy.loginfo("---")
            wrist_pos_current = 0
            judge_value = 1E-10
            wrist_pos_step = (wrist_goal - 0) / way_points
            while(abs(wrist_goal) - abs(wrist_pos_current) >= judge_value) and not rospy.is_shutdown():
                wrist_pos_current += wrist_pos_step
                wrist_pos.command = 0x01
                wrist_pos.joint = 4
                wrist_pos.position = wrist_pos_current
                self.wrist_pos_pub.publish(wrist_pos)
                rospy.sleep(0.1)
                rospy.loginfo("wrist_goal_pos:%f", wrist_goal)
                rospy.loginfo("wrist_current_pos:%f", wrist_pos_current)
                rospy.loginfo("---")
            wrist_pos_current = wrist_goal


if __name__ == '__main__':
    try:
        rospy.init_node('wrist_action_server')
        WristActionServer('xm_move_wrist')
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Exit!"
