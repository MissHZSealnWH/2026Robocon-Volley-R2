#include "Ball_back.h"
#include "RobStride2.h"
#include "Task_Init.h"
#include "step.h"
#include "motorEx.h"

extern Motor3508Ex_t Rm3508;

//RobStride_Expect R_left_expect = {
//	.expect_angle = 0.0f,
//	.expect_omega = 0.0f,
//	.expect_torque = 0.0f,
//	.kd = 0.0f,
//	.kp = 0.0f
//};
//RobStride_Expect R_right_expect = {
//	.expect_angle = 0.0f,
//	.expect_omega = 0.0f,
//	.expect_torque = 0.0f,
//	.kd = 0.0f,
//	.kp = 0.0f
//};

CubicParam_t traj_left;
CubicParam_t traj_right;

TrajectoryState_t traj_left_state;
TrajectoryState_t traj_right_state;

RobStride_Expect R_left_expect;
RobStride_Expect R_right_expect;

RobStride_t R_left;
RobStride_t R_right;

uint8_t ball_back_trigger = 0;
float time = 0.0f;
uint8_t flag = 0;
static uint8_t traj_started = 0;

TaskHandle_t Ball_back_Handle;
void Ball_back(void *pvParameters)
{
		vTaskDelay(3000);
    RobStrideInit(&R_left, &hcan1, 0x01, RobStride_MotionControl, RobStride_04);
	  RobStrideInit(&R_right, &hcan1, 0x02, RobStride_MotionControl, RobStride_04);
	  vTaskDelay(100);
	  RobStrideSetMode(&R_left, RobStride_MotionControl);
	  RobStrideSetMode(&R_right, RobStride_MotionControl);
	  vTaskDelay(100);
    RobStrideEnable(&R_left);
	  RobStrideEnable(&R_right);
	  vTaskDelay(100);

    RobStrideResetAngle(&R_left);
    RobStrideResetAngle(&R_right);
	
	
	TickType_t last_wake = xTaskGetTickCount();
	for(;;)
	{
		if (ball_back_trigger == 1 && traj_started == 0)   // 第一次触发
		{
				Cubic_SetTrajectory(
						&traj_left,
						R_left.state.rad,        // 当前真实角度
						R_left.state.omega,      // 当前真实速度
						R_left_expect.expect_angle,          // 目标角度
						0,
						time,                  
						xTaskGetTickCount()
				);

				Cubic_SetTrajectory(
						&traj_right,
						R_right.state.rad,
						R_right.state.omega,
						R_right_expect.expect_angle,
						0,
						time,
						xTaskGetTickCount()
				);
				traj_started = 1;
		}

		if(ball_back_trigger == 1)
		{
			
			Cubic_GetFullState(&traj_left,  xTaskGetTickCount(), &traj_left_state);
			Cubic_GetFullState(&traj_right, xTaskGetTickCount(), &traj_right_state);
			
			RobStrideMotionControl(&R_left, R_left.motor_id, 
			R_left_expect.expect_torque, 
			traj_left_state.pos,
			traj_left_state.vel,
			R_left_expect.kp,
			R_left_expect.kd);

			RobStrideMotionControl(&R_right, R_right.motor_id, 
			R_right_expect.expect_torque, 
			traj_right_state.pos,
			traj_right_state.vel,
			R_right_expect.kp,
			R_right_expect.kd);
			
			if (!traj_left.is_running && !traj_right.is_running)
			{
	    	RobStrideMotionControl(&R_left, R_left.motor_id,
				0.0f, traj_left.target_pos, 0.0f,
				R_left_expect.kp, R_left_expect.kd);

		    RobStrideMotionControl(&R_right, R_right.motor_id,
				0.0f, traj_right.target_pos, 0.0f,
				R_right_expect.kp, R_right_expect.kd);
			}
		}
		
		if(flag == 0 && ball_back_trigger == 0)
			{				
			RobStrideMotionControl(&R_left, R_left.motor_id, 0, R_left.state.rad, 0, 0, 0);
			RobStrideMotionControl(&R_right, R_right.motor_id, 0, R_right.state.rad, 0, 0, 0);
			}
	 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
	}
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	uint8_t buf[8];
//	uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
//	RobStrideRecv_Handle(&R_left, &hcan1, ID, buf);
//  RobStrideRecv_Handle(&R_right, &hcan1, ID, buf);
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
	RobStrideRecv_Handle(&R_left, &hcan1, ID, buf);
  RobStrideRecv_Handle(&R_right, &hcan1, ID, buf);
	
	uint8_t buff[8];
	Motor3508Recv(&Rm3508, &hcan1, ID, buf);
}