#include "Ball_back.h"
#include "RobStride2.h"
#include "Task_Init.h"

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
RobStride_Expect R_left_expect;
RobStride_Expect R_right_expect;

RobStride_t R_left;
RobStride_t R_right;

uint8_t ball_back_trigger = 0;


TaskHandle_t Ball_back_Handle;
void Ball_back(void *pvParameters)
{
		vTaskDelay(3000);
    RobStrideInit(&R_left, &hcan1, 0x01, RobStride_MotionControl, RobStride_04);
	  RobStrideInit(&R_right, &hcan1, 0x02, RobStride_MotionControl, RobStride_04);
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
		
		if(ball_back_trigger == 1)
		{
			RobStrideMotionControl(&R_left, 0x01, 
			R_left_expect.expect_torque, 
			R_left_expect.expect_angle,
			R_left_expect.expect_omega,
			R_left_expect.kp,
			R_left_expect.kd);

			RobStrideMotionControl(&R_right, 0x01, 
			R_right_expect.expect_torque, 
			R_right_expect.expect_angle,
			R_right_expect.expect_omega,
			R_right_expect.kp,
			R_right_expect.kd);
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