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

// Description: Control the arm in rviz by keyboard.

// Create Data: 2015.11.1
  
// Authors: myyerrol


#include <xm_arm_teleop/xm_keyboard_teleop_rviz_arm.h>


int kfd = 0;
struct termios cooked, raw;


KeyboardTeleopRvizArm::KeyboardTeleopRvizArm()
{
    ros::NodeHandle n_private("~");
    n_private.param("joint_state_step", joint_state_step_, 0.01745);

    joint_state_pub_ = xm_nh_.advertise<sensor_msgs::JointState>("joint_states", 1000);
}


KeyboardTeleopRvizArm::~KeyboardTeleopRvizArm()
{

}


void KeyboardTeleopRvizArm::StopArm()
{

}

void KeyboardTeleopRvizArm::ArmLoop()
{
    char keyboard_cmd;
    bool use_flag = false;
    memset(goal_joint_position_, 0, sizeof(goal_joint_position_));
    memset(rviz_joint_position_, 0, sizeof(rviz_joint_position_));
    joint_dof_ = 6;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("-----------------------------------");
    puts("      Move The Arm By KeyBoard     ");
    puts("-----------------------------------");
    puts("Q                W                E");
    puts("A                                 D");
    puts("                 S                 ");
    puts("-----------------------------------");
    puts("W:Lift-UP          S:Waist-L       ");
    puts("A:Big_Arm-UP       D:Forearm-UP    ");
    puts("Q:Wrist_Pitching-UP                ");
    puts("E:Wrist_Rotation-CLOCK++           ");
    puts("-----------------------------------");
    puts("Shift+W:Lift-DOWN  Shift+S:Waist-R ");
    puts("Shift+A:Big_Arm-DOWN               ");
    puts("Shift+D:Forearm-DOWN               ");
    puts("Shift+Q:Wrist_Pitching-DOWN        ");
    puts("Shift+E:Wrist_Rotation-CLOCK--     ");
    puts("-----------------------------------");
    puts("QUIT/CTRL-C TO QUIT                ");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();
        // get the next event from the keyboard
        int num;
        if((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &keyboard_cmd, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else if(use_flag == true)
            continue;
        switch(keyboard_cmd)
        {
            case KEYCODE_W:
                joint_state_step_ = 0.01;
                goal_joint_position_[0] += joint_state_step_;
                joint_state_step_ = 0.01745;
                if(goal_joint_position_[0] >= 0.20)
                    goal_joint_position_[0] = 0.20;
                break;
            case KEYCODE_S:
                goal_joint_position_[1] += joint_state_step_;
                if(goal_joint_position_[1] >= 1.047)
                    goal_joint_position_[1] = 1.047;
                break;
            case KEYCODE_A:
                goal_joint_position_[2] += joint_state_step_;
                if(goal_joint_position_[2] >= 1.309)
                    goal_joint_position_[2] = 1.309;
                break;
            case KEYCODE_D:
                goal_joint_position_[3] += joint_state_step_;
                if(goal_joint_position_[3] >= 2.234)
                    goal_joint_position_[3] = 2.234;
                break;
            case KEYCODE_Q:
                goal_joint_position_[4] += joint_state_step_;
                if(goal_joint_position_[4] >= 2.182)
                    goal_joint_position_[4] = 2.182;
                break;
            case KEYCODE_E:
                goal_joint_position_[5] += joint_state_step_;
                break;
            case KEYCODE_W_CAP:
                joint_state_step_ = 0.01;
                goal_joint_position_[0] -= joint_state_step_;
                joint_state_step_ = 0.0174;
                if(goal_joint_position_[0] <= -0.20)
                    goal_joint_position_[0] = -0.20;
                break;
            case KEYCODE_S_CAP:
                goal_joint_position_[1] -= joint_state_step_;
                if(goal_joint_position_[1] <= -1.047)
                    goal_joint_position_[1] = -1.047;
                break;
            case KEYCODE_A_CAP:
                goal_joint_position_[2] -= joint_state_step_;
                if(goal_joint_position_[2] <= -1.396)
                    goal_joint_position_[2] = -1.396;
                break;
            case KEYCODE_D_CAP:
                goal_joint_position_[3] -= joint_state_step_;
                if(goal_joint_position_[3] <= -2.234)
                    goal_joint_position_[3]  = -2.234;
                break;
            case KEYCODE_Q_CAP:
                goal_joint_position_[4] -= joint_state_step_;
                if(goal_joint_position_[4] <= -2.182)
                    goal_joint_position_[4]  = -2.182;
                break;
            case KEYCODE_E_CAP:
                goal_joint_position_[5] -= joint_state_step_;
                break;
            default: ;
        }
        use_flag = true;
        SendJointState();
    }
}


void KeyboardTeleopRvizArm::SendJointState()
{
    rviz_joint_position_[0] = -goal_joint_position_[0];
    rviz_joint_position_[1] =  goal_joint_position_[1];
    rviz_joint_position_[2] =  goal_joint_position_[2];
    rviz_joint_position_[3] = -goal_joint_position_[3];
    rviz_joint_position_[4] = -goal_joint_position_[4];
    rviz_joint_position_[5] =  goal_joint_position_[5];

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(joint_dof_);
    joint_state.name[0] = "joint_lift";
    joint_state.name[1] = "joint_waist";
    joint_state.name[2] = "joint_big_arm";
    joint_state.name[3] = "joint_forearm";
    joint_state.name[4] = "joint_wrist_pitching";
    joint_state.name[5] = "joint_wrist_rotation";
    joint_state.position.resize(joint_dof_);
    joint_state.velocity.resize(joint_dof_);
    joint_state.effort.resize(joint_dof_);
    for(int i = 0; i < joint_dof_; i++)
    {
        joint_state.position[i] = rviz_joint_position_[i];
        joint_state.velocity[i] = 0;
        joint_state.effort[i] = 0;
    }
    joint_state_pub_.publish(joint_state);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "xm_keyboard_teleop_rviz_arm", ros::init_options::NoSigintHandler);
    KeyboardTeleopRvizArm tbk;
    boost::thread t = boost::thread(boost::bind(&KeyboardTeleopRvizArm::ArmLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.StopArm();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}
