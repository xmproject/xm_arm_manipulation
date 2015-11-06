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

// Description: Control the arm by keyboard.

// Create Date: 2015.11.1
  
// Authors: myyerrol  


#include <xm_arm_teleop/xm_keyboard_teleop_arm.h>


int kfd = 0;
struct termios cooked, raw;


KeyboardTeleopArm::KeyboardTeleopArm()
{
    ros::NodeHandle n_private("~");
    n_private.param("joint_angular_step", joint_angular_step_, 0.0174);
    n_private.param("gripper_angular_step", gripper_angular_step_, 0.1);

    joint_pos_pub_ = xm_nh_.advertise<xm_msgs::xm_JointPos>("joint_pos_cmd", 1000);
    gripper_pos_pub_ = xm_nh_.advertise<std_msgs::Float64>("gripper_joint/command", 1000);
}


KeyboardTeleopArm::~KeyboardTeleopArm()
{

}


void KeyboardTeleopArm::StopArm()
{

}


void KeyboardTeleopArm::ArmLoop()
{
    char keyboard_cmd;
    int joint_number;
    double joint_positon[6];
    double gripper_position = 0;
    memset(joint_positon, 0, sizeof(joint_positon));

    bool dirty = false;
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
    puts("                 X                 ");
    puts("-----------------------------------");
    puts("W:Lift-UP          S:Waist-L       ");
    puts("A:Big_Arm-UP       D:Forearm-UP    ");
    puts("Q:Wrist_Pitching-UP                ");
    puts("E:Wrist_Rotation-CLOCK++           ");
    puts("X:Gripper-Open                     ");
    puts("-----------------------------------");
    puts("Shift+W:Lift-DOWN  Shift+S:Waist-R ");
    puts("Shift+A:Big_Arm-DOWN               ");
    puts("Shift+D:Forearm-DOWN               ");
    puts("Shift+Q:Wrist_Pitching-DOWN        ");
    puts("Shift+E:Wrist_Rotation-CLOCK--     ");
    puts("Shift+X:Gripper-Close              ");
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
        else
        {
            if(dirty == true)
                continue;
        }
        switch(keyboard_cmd)
        {
            case KEYCODE_W:
                joint_number = 0;
                joint_angular_step_ = 0.01;
                joint_positon[0] += joint_angular_step_;
                joint_angular_step_ = 0.0174;
                if(joint_positon[0] >= 0.20)
                    joint_positon[0] = 0.20;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[0];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_S:
                joint_number = 1;
                joint_positon[1] += joint_angular_step_;
                if(joint_positon[1] >= 1.047)
                    joint_positon[1] = 1.047;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[1];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_A:
                joint_number = 2;
                joint_positon[2] += joint_angular_step_;
                if(joint_positon[2] >= 1.309)
                    joint_positon[2] = 1.309;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[2];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_D:
                joint_number = 3;
                joint_positon[3] += joint_angular_step_;
                if(joint_positon[3] >= 2.234)
                    joint_positon[3] = 2.234;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[3];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_Q:
                joint_number = 4;
                joint_positon[4] += joint_angular_step_;
                if(joint_positon[4] >= 2.182)
                    joint_positon[4] = 2.182;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[4];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_E:
                joint_number = 5;
                joint_positon[5] += joint_angular_step_;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[5];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_W_CAP:
                joint_number = 0;
                joint_angular_step_ = 0.01;
                joint_positon[0] -= joint_angular_step_;
                joint_angular_step_ = 0.0174;
                if(joint_positon[0] <= -0.20)
                    joint_positon[0] = -0.20;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[0];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_S_CAP:
                joint_number = 1;
                joint_positon[1] -= joint_angular_step_;
                if(joint_positon[1] <= -1.047)
                    joint_positon[1] = -1.047;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[1];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_A_CAP:
                joint_number = 2;
                joint_positon[2] -= joint_angular_step_;
                if(joint_positon[2] <= -1.396)
                    joint_positon[2] = -1.396;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[2];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_D_CAP:
                joint_number = 3;
                joint_positon[3] -= joint_angular_step_;
                if(joint_positon[3] <= -2.234)
                    joint_positon[3]  = -2.234;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[3];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_Q_CAP:
                joint_number = 4;
                joint_positon[4] -= joint_angular_step_;
                if(joint_positon[4] <= -2.182)
                    joint_positon[4]  = -2.182;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[4];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_E_CAP:
                joint_number = 5;
                joint_positon[5] -= joint_angular_step_;
                dirty = true;
                joint_pos_.command  = 0x01;
                joint_pos_.joint    = joint_number;
                joint_pos_.position = joint_positon[5];
                joint_pos_pub_.publish(joint_pos_);
                break;
            case KEYCODE_X:
                gripper_position += gripper_angular_step_;
                if(gripper_position >= 1.0)
                    gripper_position = 1.0;
                dirty = true;
                gripper_pos_.data = gripper_position;
                gripper_pos_pub_.publish(gripper_pos_);
                break;
            case KEYCODE_X_CAP:
                gripper_position -= gripper_angular_step_;
                if(gripper_position <= -1.0)
                    gripper_position = -1.0;
                dirty = true;
                gripper_pos_.data = gripper_position;
                gripper_pos_pub_.publish(gripper_pos_);
                break;
            default:
                dirty = false;
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"xm_keyboard_teleop_arm", ros::init_options::NoSigintHandler);
     KeyboardTeleopArm tbk;

    boost::thread t = boost::thread(boost::bind(&KeyboardTeleopArm::ArmLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.StopArm();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}
