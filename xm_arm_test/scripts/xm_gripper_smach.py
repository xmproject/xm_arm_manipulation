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

# Description: This script use the state machine to test the gripper's open or 
# close.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
from smach import State, StateMachine
from smach_ros import SimpleActionState
from xm_msgs.msg import xm_GripperCommandAction, xm_GripperCommandGoal


class TestGripperSmach:
    def __init__(self):
        rospy.init_node('test_gripper_smach', anonymous=False)
        gripper_goal = xm_GripperCommandGoal()
        gripper_goal.command.position = 0.10
        gripper_goal.command.torque = 0.5
        gripper_state = SimpleActionState('xm_move_gripper', xm_GripperCommandAction, goal=gripper_goal,
                                          result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(10),
                                          server_wait_timeout=rospy.Duration(10))
        self.gripper_smach = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with self.gripper_smach:
            StateMachine.add('GRIPPER_OPEN', gripper_state,
                             transitions={'succeeded': '', 'aborted': '', 'preempted': ''})
        gripper_outcome = self.gripper_smach.execute()
        rospy.loginfo('State Machine Outcome: ' + str(gripper_outcome))

    def gripper_state_cb(self, userdata, status, result):
        if result:
            rospy.loginfo("Complete!")


if __name__ == '__main__':
    try:
        TestGripperSmach()
    except rospy.ROSInterruptException:
        rospy.logerr("Exit!")
