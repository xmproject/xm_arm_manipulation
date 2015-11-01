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

# Description: This script bring up the action's client to test gripper's open or
# close with user's interactional input.

# Create Date: 2015.11.1

# Authors: myyerrol  


import rospy
from std_msgs.msg import Float64
from sys import exit
from math import sin, cos, sqrt
from scipy.optimize import fsolve


class TestGripperMove:
    def __init__(self):
        rospy.init_node('gripper_test', anonymous=True)
        gripper_pos_pub = rospy.Publisher('gripper_joint/command', Float64, queue_size=1000)
        print "------------ Test Gripper Accurate Move ------------"
        print "Input servo's position to compute gripper's position"
        print "Input gripper's position to compute servo's position"
        while not rospy.is_shutdown():
            print ">>> ",
            keyboard_cmd = raw_input().split(" ")
            try:
                if keyboard_cmd[0] == "gripper":
                    if -1.0 <= float(keyboard_cmd[1]) <= 1.5:
                        theta = float(keyboard_cmd[1])
                        position = 2 * (2.5 * sin(theta) + sqrt(4.5 ** 2 - 2.5 ** 2 * cos(theta) ** 2) - 2)
                        print "object_length:%lfm" % (position / 100)
                    else:
                        print "The number must between -1.0 and 1.5"
                elif keyboard_cmd[0] == "object":
                    def object_position(x):
                        offset = 1.50
                        length = float(keyboard_cmd[1]) * 100 - offset
                        theta = float(x[0])
                        return [
                            length - 2 * (2.5 * sin(theta) + sqrt(4.5 ** 2 - 2.5 ** 2 * cos(theta) ** 2) - 2)
                        ]
                    servo_position = fsolve(object_position, [1])
                    print "servo_theta:%lfrad" % servo_position
                    gripper_pos_pub.publish(servo_position)
                elif keyboard_cmd[0] == "exit":
                    exit()
            except Exception as exce:
                print "Error!", exce


if __name__ == '__main__':
    try:
        gripper_move = TestGripperMove()
    except KeyboardInterrupt:
        print "Exit!"
