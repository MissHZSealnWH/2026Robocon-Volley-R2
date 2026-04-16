#include "Chassis.h"
#include "VESC.h"
#include "PID_old.h"
#include "Task_Init.h"
#include "semphr.h"
#include "dataFrame.h"
#include "comm.h"
#include "comm_stm32_hal_middle.h"
#include "data_poll.h"
#include "My_list.h"
#include "math.h"

extern SemaphoreHandle_t remote_semaphore;

//遥控器
PackControl_t recv_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
uint8_t usart5_buff[30];

//电机驱动
Motor_param motor1 = {
.PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 40.0f,
},
.steering={
	.motor_id=0x01,
	.hfdcan = &hfdcan1,
}
};
Motor_param motor2 = {
.PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 40.0f,
},
.steering={
	.motor_id=0x02,
	.hfdcan = &hfdcan1,
}
};
Motor_param motor3 = {
.PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 40.0f,
},
.steering={
	.motor_id=0x03,
	.hfdcan = &hfdcan1,
}
};

//遥控模式
Positon_label MODE = REMOTE;

volatile float Vx =0;   //前后移动
volatile float Vy =0;   //左右移动
volatile float Wz =0;   //顺逆自转

volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;

volatile float wheel_one = 0.0f;
volatile float wheel_two = 0.0f;
volatile float wheel_three=0.0f;


static void Key_Parse(uint32_t key, hw_key_t *out)
{
  	out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}

//void Remote_Analysis()
//{
//	/* 1. 保存上一帧 */
//	Remote_Control.Second = Remote_Control.First;
//	/* 2. 解析当前按键 */
//	Key_Parse(recv_pack.Key, &Remote_Control.First);
//	
//	Remote_Control.Ex = recv_pack.rocker[1] / 1977.0f *MAX_ROBOT_VEL;
//	Remote_Control.Ey = recv_pack.rocker[0] / 1798.0f *MAX_ROBOT_VEL;
//	Remote_Control.Eomega = recv_pack.rocker[2] / 1847.0f * MAX_ROBOT_OMEGA;
//}

void Remote_Analysis()
{
    if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      /* 1. 保存上一帧 */
      Remote_Control.Second = Remote_Control.First;
      /* 2. 解析当前按键 */
      Key_Parse(recv_pack.Key, &Remote_Control.First);
			Remote_Control.Ex = recv_pack.rocker[1] / 1977.0f *MAX_ROBOT_VEL;
			Remote_Control.Ey = recv_pack.rocker[0] / 1798.0f *MAX_ROBOT_VEL;
			Remote_Control.Eomega = recv_pack.rocker[2] / 1847.0f * MAX_ROBOT_OMEGA;
    }else {
	    Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;
			
      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
    }
}

//遥控器滤波降噪 
void Rocker_Filter(PackControl_t *data)
{
    float alpha = 0.6f;

    for(int i = 0; i < 4; i++)
    {
        rocker_filter[i] = alpha * data->rocker[i] + (1.0f - alpha) * rocker_filter[i];

        data->rocker[i] = rocker_filter[i];
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    Rocker_Filter(&recv_pack);
		//遥控器
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


CommPackRecv_Cb recv_cb = MyRecvCallback;

TaskHandle_t Remote_Handle;
void Remote(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();

	  g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
	for(;;)
	{

		if(MODE == REMOTE)
		{			
			Remote_Analysis();
			v1 = -Vy*0.5f+Vx*(sqrtf(3.0f)/2.0f) + R * Wz;
			v2 = -Vy*0.5f-Vx*(sqrtf(3.0f)/2.0f) + R * Wz;
			v3 = Vy + R * Wz;			
			
			wheel_one=  -((v1 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_two = (( v2 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_three=-((v3 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			
			PID_Control2((float)(((float)motor1.steering.epm / 7.0f/(3.4f))), wheel_one, &motor1.PID);
      PID_Control2((float)(((float)motor2.steering.epm / 7.0f/(3.4f))), wheel_two, &motor2.PID);
			PID_Control2((float)(((float)motor3.steering.epm / 7.0f/(3.4f))), wheel_three, &motor3.PID);
			
      VESC_SetCurrent(&motor1.steering, motor1.PID.pid_out);
      VESC_SetCurrent(&motor2.steering, motor2.PID.pid_out);
	    VESC_SetCurrent(&motor3.steering, motor3.PID.pid_out);  
			
		if(recv_pack.rocker[0] == 0 && recv_pack.rocker[1] == 0 && recv_pack.rocker[2] == 0 )
//				if(abs(recv_pack.rocker[3]>1500))第二判断法
			{
	    Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;
			
      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
			
		 	}
		}
		if(MODE == STP || MODE == STOP )
		{
			wheel_one = 0;
			wheel_two = 0;
			wheel_three=0;
			
			VESC_SetCurrent(&motor1.steering, 0);
			VESC_SetCurrent(&motor2.steering, 0);
			VESC_SetCurrent(&motor3.steering, 0);
		}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
	}
}

//遥控器任务
TaskHandle_t Control_Remote_Handle;
void Control_Remote(void *pvParameters)
{
	for(;;)
	{
		Remote_Analysis();
	 }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if(hfdcan->Instance==FDCAN1 && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
	uint8_t Recv[8];
	uint32_t ID;
	FDCAN_Receive_DataFrame(&hfdcan1, &ID, Recv, NULL, NULL);
	VESC_ReceiveHandler(&motor1.steering, &hfdcan1, ID, Recv);
	VESC_ReceiveHandler(&motor2.steering, &hfdcan1, ID, Recv);
	VESC_ReceiveHandler(&motor3.steering, &hfdcan1, ID, Recv);
  }
}
