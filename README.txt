# xm_arm_manipulation

Create Date: 2015.11.1

Authors: myyerrol

Function: 
This metapackage implements the first generation of our xmbot's arm manipulation.

Package:
1.xm_arm_bringup: Bring up arm or gripper's controllers and other default parameters to initialize states.
2.xm_arm_control: This package implements some important componets about joint's action and service to provide interfaces with state machine.
3.xm_arm_teleop: Control arm with gripper by using keyboard.
4.xm_arm_test: Test arm's manipulation.

Summary:
In this version, though whole arm's pick and place pipeline only uses four joint's dof instead of six, we achieve a simple arm manipulation with our xmbot successfully! In the future version, in lower layer, we plan to rewrite arm's underlying codes to make more fiexible control and encapsulate codes for being called by statemachine more easily. And for upper layer, we will use moveit package to achieve simple motion planning. To sum up, next version can implement complete arm manipulation really.
