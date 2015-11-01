#ifndef KEYBOARD_TELEOP_GRIPPER_H
#define KEYBOARD_TELEOP_GRIPPER_H

#include <termio.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/thread/thread.hpp>

#define KEYCODE_W 0X77
#define KEYCODE_S 0X73


class KeyboardTeleopGripper
{
public:
     KeyboardTeleopGripper();
    ~ KeyboardTeleopGripper();
    void GripperLoop();
    void StopGripper();
protected:
private:
    ros::NodeHandle xm_nh_;
    ros::Publisher gripper_pos_pub_;

    std_msgs::Float64 gripper_pos_;
    double gripper_angular_;
    double gripper_angular_step_;
};

#endif //KEYBOARD_TELEOP_GRIPPER_H
