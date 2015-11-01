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

# Description: This script bring up the action's client to control the position 
# of wrist's pitching.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
import actionlib
from xm_msgs.msg import xm_WristPositionAction, xm_WristPositionGoal


def wrist_action_client():

    wrist_client = actionlib.SimpleActionClient('xm_move_wrist', xm_WristPositionAction)

    rospy.loginfo("Waiting for xm_move_wrist action server")
    wrist_client.wait_for_server()
    rospy.loginfo("Connected to xm_move_wrist action server")

    wrist_goal = xm_WristPositionGoal()
    wrist_goal.angle = 0

    rospy.loginfo("Preparing for sending goal to the action server")
    wrist_client.send_goal(wrist_goal)
    wrist_client.wait_for_result(rospy.Duration(0))
    rospy.loginfo("Send goal to the action server successfully!")

    return wrist_client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('wrist_action_client')
        result = wrist_action_client()
        print result
    except rospy.ROSInterruptException:
        print "Error!"
