#!/usr/bin/env python


import rospy
import actionlib
from xm_msgs.msg import xm_LiftHeightAction, xm_LiftHeightGoal


def lift_action_client():
    lift_client = actionlib.SimpleActionClient('xm_move_lift', xm_LiftHeightAction)
    rospy.loginfo("Waiting for xm_move_lift action server")
    lift_client.wait_for_server()
    rospy.loginfo("Connected to xm_move_lift action server")

    lift_goal = xm_LiftHeightGoal()
    lift_goal.height = 0.05

    rospy.loginfo("Preparing for sending goal to the action server")
    lift_client.send_goal(lift_goal)
    lift_client.wait_for_result(rospy.Duration(0))
    rospy.loginfo("Send goal to the action server successfully!")

    return lift_client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('lift_height_action_client')
        result = lift_action_client()
        print result
    except rospy.ROSInterruptException:
        print "Error!"
