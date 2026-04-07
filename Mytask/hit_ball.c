#include "hit_ball.h"
#include "go_motor.h"
#include "485_bus.h"

RS485_t rs485bus;

Exp_param Exp_3508;
Rm3508 rm3508 = {
	.pos_pid_3508 = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 500.0f,
		.output_limit = 10000.0f
			},
	.vel_pid_3508 = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 500.0f,
		.output_limit = 10000.0f
			}
	};

Unitreecontrol ball_Start = {
	.Go_volleyball.motor_id = 0x01,
	.Go_volleyball.rs485 = &rs485bus
};
	
	uint32_t error_cnt = 0;
	uint32_t last_error_time = 0;
	ErrorStats_t error_stats = {0};
	Exp_param go_volley = {0};
	uint32_t err_timer_cnt = 0;

	
TaskHandle_t Hit_Task_Handle;
void Hit_Task(void *pvParameters)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	
		//3508reset
	  int16_t reset_command[4];
	  //3508send
		int16_t can_send_buf[4];

		for(;;)
		{
			PID_Control2(rm3508.motor_3508.motor.MchanicalAngle, Exp_3508.exp_pos, &rm3508.pos_pid_3508);
			PID_Control2(rm3508.motor_3508.motor.Speed, rm3508.pos_pid_3508.pid_out, &rm3508.vel_pid_3508);
			
			int16_t data = (int16_t)rm3508.vel_pid_3508.pid_out;
			can_send_buf[0] = data;
			//默认id为3508id为1
			MotorSend(&hcan1, 0x200, can_send_buf);
			vTaskDelay(350);
			//宇树
			GoMotorSend(&ball_Start.Go_volleyball,
			ball_Start.Exp.exp_torque,
			ball_Start.Exp.exp_vel,
			ball_Start.Exp.exp_pos,
			ball_Start.Exp.exp_kp,
			ball_Start.Exp.exp_kd);
			
			GoMotorRecv(&ball_Start.Go_volleyball);
//			vTaskDelay(5000);
			
//			GoMotorSend();
//			vTaskDelay(1000);
      //3508复位
//      int16_t rm3508_reset;
//			
//			reset_command[0] = rm3508_reset;
//			
//			MotorSend(&hcan1,0x200,reset_command);

		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
			}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t buf[8];
    uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
    Motor3508Recv(&rm3508.motor_3508, &hcan1, ID, buf);
}

