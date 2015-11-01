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
# lift independently.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
import actionlib
from xm_msgs.msg import xm_LiftHeightResult, xm_LiftHeightFeedback
from xm_msgs.msg import xm_LiftHeightAction
from xm_msgs.msg import xm_JointPos


class LiftActionServer(object):
    result = xm_LiftHeightResult()
    feedback = xm_LiftHeightFeedback()

    def __init__(self, name):
        self.lift_action_name = name
        self.lift_action_server = actionlib.SimpleActionServer(self.lift_action_name, xm_LiftHeightAction,
                                                               execute_cb=self.execute_cb, auto_start=False)
        self.lift_action_server.start()
        rospy.loginfo("Start action server!")

    def execute_cb(self, goal):
        rate = rospy.Rate(20)
        success = True

        rospy.loginfo("Receiving the lift_joint goal: %lf", goal.height)

        if self.lift_action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.lift_action_name)
            self.lift_action_server.set_preempted()
            success = False

        rate.sleep()
        self.send_lift_goal(goal.height)

        self.feedback.height = goal.height
        self.lift_action_server.publish_feedback(self.feedback)

        if success:
            self.result.complete = True
            rospy.loginfo("Succeeded!!")
            self.lift_action_server.set_succeeded(self.result)

    def send_lift_goal(self, lift_goal):
        lift_pos_pub = rospy.Publisher('/joint_pos_cmd', xm_JointPos, queue_size=1000)
        lift_pos = xm_JointPos()
        lift_pos.command = 0x01
        lift_pos.joint = 0
        lift_pos.position = lift_goal
        lift_pos_pub.publish(lift_pos)
        rospy.loginfo("Publish lift's height to the topic!")


if __name__ == '__main__':
    try:
        rospy.init_node('lift_action_server')
        LiftActionServer('xm_move_lift')
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Error"
