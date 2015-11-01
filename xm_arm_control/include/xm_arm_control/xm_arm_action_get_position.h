#ifndef ARM_ACTION_GET_POSITION_H
#define ARM_ACTION_GET_POSITION_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <xm_msgs/xm_JointPos.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/JointTrajectoryControllerState.h>


class ArmActionGetPosition
{
public:
    ArmActionGetPosition();
    ~ArmActionGetPosition();
protected:
private:
    void JointPosCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
private:
    ros::NodeHandle xm_nh_;
    ros::Publisher  joint_pos_pub_;
    ros::Publisher joint_pub_;
    ros::Subscriber joint_pos_sub_;
    std::vector<double> joint_position_;
    std::vector<double> joint_position_current_;
    std::vector<double> trajectory_duration_;
    int joint_num_;
    int trajectory_num_;
    int waist_switch_flag_;
    int wrist_switch_flag_;
};

#endif //ARM_ACTION_GET_POSITION_H
