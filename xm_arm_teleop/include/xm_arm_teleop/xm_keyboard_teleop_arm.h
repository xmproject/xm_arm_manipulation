#ifndef KEYBOARD_TELEOP_ARM_H
#define KEYBOARD_TELEOP_ARM_H

#include <termio.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <ros/ros.h>
#include <xm_msgs/xm_JointPos.h>
#include <std_msgs/Float64.h>
#include <boost/thread/thread.hpp>

#define KEYCODE_A 0X61
#define KEYCODE_D 0X64
#define KEYCODE_S 0X73
#define KEYCODE_W 0X77
#define KEYCODE_Q 0X71
#define KEYCODE_E 0X65
#define KEYCODE_X 0x78

#define KEYCODE_A_CAP 0X41
#define KEYCODE_D_CAP 0X44
#define KEYCODE_S_CAP 0X53
#define KEYCODE_W_CAP 0X57
#define KEYCODE_Q_CAP 0X51
#define KEYCODE_E_CAP 0X45
#define KEYCODE_X_CAP 0x58


class KeyboardTeleopArm
{
public:
    KeyboardTeleopArm();
    ~KeyboardTeleopArm();
    void ArmLoop();
    void StopArm();
protected:
private:
    ros::NodeHandle xm_nh_;
    ros::Publisher joint_pos_pub_;
    ros::Publisher gripper_pos_pub_;

    xm_msgs::xm_JointPos joint_pos_;
    std_msgs::Float64 gripper_pos_;

    double joint_angular_step_;
    double gripper_angular_step_;
};

#endif //KEYBOARD_TELEOP_ARM_H
