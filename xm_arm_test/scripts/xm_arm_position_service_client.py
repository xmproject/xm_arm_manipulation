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

# Description: This script bring up the action's client to test arm's positions.

# Create Date: 2015.11.1

# Authors: myyerrol  


import sys
import rospy
from xm_msgs.srv import xm_ArmPosition


class ArmPositionServiceClient:
    def __init__(self):
        rospy.init_node('arm_service_client', anonymous=True)
        if len(sys.argv) == 2:
            position = sys.argv[1]
            rospy.loginfo("Position:%s", position)
        else:
            print self.usage()
            sys.exit(1)
        rospy.loginfo("Preparing for sending arm position to the service server")
        self.arm_position_client(position)

    def arm_position_client(self, arm_position):
        name = 'xm_move_arm_position'
        rospy.wait_for_service(name + '/' + 'move_position')
        rospy.loginfo("Connected to the service server")
        try:
            arm_service_client = rospy.ServiceProxy(name + '/' + 'move_position', xm_ArmPosition)
            response = arm_service_client(arm_position)
            rospy.loginfo("Reponse: %d", response.complete)
        except rospy.ServiceException, exce:
            rospy.logerr("Service call failed: %s" % exce)

    def usage(self):
        return "%s [position]" % sys.argv[0]


if __name__ == '__main__':
    try:
        arm_position_client = ArmPositionServiceClient()
    except rospy.ROSInterruptException:
        print "Exit!"
