#ifndef __BALL_BACK_H_
#define __BALL_BACK_H_

#include "PID_old.h"



typedef struct
{
  float expect_torque;
	float expect_angle;
	float expect_omega;
//motioncontrol再用
	float kp;
	float kd;
}RobStride_Expect;


//左电机
typedef struct
{
	PID2 pos_pid;
	PID2 speed_pid;
}R_left_PID;

//右电机
typedef struct
{
	PID2 pos_pid;
	PID2 speed_pid;
}R_right_PID;

#endif



