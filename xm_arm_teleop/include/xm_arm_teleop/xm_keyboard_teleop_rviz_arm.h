#ifndef KEYBOARD_TELEOP_RVIZ_ARM_H
#define KEYBOARD_TELEOP_RVIZ_ARM_H

#include <termio.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

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


class KeyboardTeleopRvizArm
{
public:
    KeyboardTeleopRvizArm();
    ~KeyboardTeleopRvizArm();
    void ArmLoop();
    void StopArm();
protected:
private:
    void SendJointState();
private:
    ros::NodeHandle xm_nh_;
    ros::Publisher joint_state_pub_;

    double goal_joint_position_[6];
    double rviz_joint_position_[6];
    double joint_state_step_;
    int joint_dof_;
};
#endif //KEYBOARD_TELEOP_RVIZ_ARM_H
