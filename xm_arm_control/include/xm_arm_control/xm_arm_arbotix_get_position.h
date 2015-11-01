#ifndef ARM_ARBOTIX_GET_POSITION_H
#define ARM_ARBOTIX_GET_POSITION_H

#include <ros/ros.h>
#include <sstream>
#include <vector>
#include <string>
#include <xm_msgs/xm_JointPos.h>
#include <sensor_msgs/JointState.h>


class ArmArbotixGetPosition
{
public:
    ArmArbotixGetPosition();
    ~ArmArbotixGetPosition();
protected:
private:
    void JointPosCallback(const sensor_msgs::JointState::ConstPtr &msg);
private:
    ros::NodeHandle xm_nh_;
    ros::Publisher joint_pos_pub_;
    ros::Subscriber joint_pos_sub_;
    xm_msgs::xm_JointPos xm_joint_pos_;
    int joint_dof_;
    std::vector<double> joint_position_;
};

#endif //ARM_ARBOTIX_GET_POSITION_H
