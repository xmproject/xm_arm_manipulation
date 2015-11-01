#ifndef ARM_INIT_TRAJECTORY_H
#define ARM_INIT_TRAJECTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;


class ArmInitTrajectory
{
public:
    ArmInitTrajectory();
    ~ArmInitTrajectory();
    control_msgs::FollowJointTrajectoryGoal ArmTrajectory();
    void SendTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
    actionlib::SimpleClientGoalState GetState();
protected:
private:
    ros::NodeHandle xm_nh_;
    boost::shared_ptr<TrajectoryClient> trajectory_client_;
    int trajectory_index_;
};

#endif //ARM_INIT_TRAJECTORY_H
