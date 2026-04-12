#include "Ball_back.h"
#include "RobStride2.h"
#include "Task_Init.h"
#include "step.h"
#include "motorEx.h"

extern RM3508_TypeDef Rm3508;

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

RobStride_Reset R_left_reset;
RobStride_Reset R_right_reset;

RobStride_t R_left;
RobStride_t R_right;

uint8_t ball_back_trigger = 0;
uint8_t flag = 0;
static uint8_t traj_started = 0;
uint8_t the_symbol = 0;
static GPIO_PinState key1, key2, key3, key4;
float time = 0.0f;

TaskHandle_t Ball_back_Handle;
void Ball_back(void *pvParameters)
{
		vTaskDelay(3000);
    RobStrideInit(&R_left, &hcan1, 0x01, RobStride_04);
	  RobStrideInit(&R_right, &hcan1, 0x02, RobStride_04);
	  vTaskDelay(100);
	  RobStrideSetMode(&R_left, RobStride_MotionControl);
	  RobStrideSetMode(&R_right, RobStride_MotionControl);
	  vTaskDelay(100);
    RobStrideEnable(&R_left);
	  RobStrideEnable(&R_right);
	  vTaskDelay(100);

    RobStrideResetAngle(&R_left);
    RobStrideResetAngle(&R_right);
	
	  R_left_reset.reset_angle = R_left.state.rad;
	  R_right_reset.reset_angle = R_right.state.rad;
	
	TickType_t last_wake = xTaskGetTickCount();
	for(;;)
	{
		key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		key3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		key4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
		if(key1 == GPIO_PIN_SET || key2 == GPIO_PIN_SET || key3 == GPIO_PIN_SET || key4 == GPIO_PIN_SET)
			{
				the_symbol = 1;
				ball_back_trigger = 1;
			}
	if(the_symbol == 1)
	{
		if (ball_back_trigger == 1 && traj_started == 0)   // 一次触发
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
			
			RobStrideMotionControl(&R_left, 0x01, 
			R_left_expect.expect_torque, 
			traj_left_state.pos,
			traj_left_state.vel,
			R_left_expect.kp,
			R_left_expect.kd);

			RobStrideMotionControl(&R_right, 0x02, 
			R_right_expect.expect_torque, 
			traj_right_state.pos,
			traj_right_state.vel,
			R_right_expect.kp,
			R_right_expect.kd);
	}
		vTaskDelay(300);
	
	 		RobStrideMotionControl(&R_left, 0x01, 
			R_left_reset.reset_torque, 
			R_left_reset.reset_angle,
			R_left_reset.reset_omega,
			R_left_reset.kp,
			R_left_reset.kd);

	 		RobStrideMotionControl(&R_left, 0x01, 
			R_right_reset.reset_torque, 
			R_right_reset.reset_angle,
			R_right_reset.reset_omega,
			R_right_reset.kp,
			R_right_reset.kd);
	
	   traj_started = 0;
	
	    vTaskDelay(200);
		
// 此处仅测试使用	
//		if(flag == 0 && ball_back_trigger == 0)
//			{				
//			RobStrideMotionControl(&R_left, 0x01, 0, R_left.state.rad, 0, 0, 0);
//			RobStrideMotionControl(&R_right, 0x02, 0, R_right.state.rad, 0, 0, 0);
//			}
		}
	 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
	RobStrideRecv_Handle(&R_left, &hcan1, ID, buf);
  RobStrideRecv_Handle(&R_right, &hcan1, ID, buf);
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	uint8_t buf[8];
//	uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
//	RobStrideRecv_Handle(&R_left, &hcan1, ID, buf);
//  RobStrideRecv_Handle(&R_right, &hcan1, ID, buf);
//	
//	uint8_t buff[8];
//	RM3508_Receive(&Rm3508, buff);
//}