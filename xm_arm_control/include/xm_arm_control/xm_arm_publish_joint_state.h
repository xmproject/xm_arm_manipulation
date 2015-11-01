#ifndef ARM_PUBLISH_JOINT_STATE_H
#define ARM_PUBLISH_JOINT_STATE_H

#include <ros/ros.h>
#include <sstream>
#include <vector>
#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>


class ArmPublishJointState
{
public:
    ArmPublishJointState();
    ~ArmPublishJointState();
protected:
private:
    void JointPosCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
private:
    ros::NodeHandle xm_nh_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber joint_pos_sub_;
    std::vector<double> joint_position_;
    int joint_dof_;
};

#endif //ARM_PUBLISH_JOINT_STATE_H
