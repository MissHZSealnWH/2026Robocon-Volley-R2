#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "hit_ball.h"

extern RS485_t rs485bus;
extern uint8_t usart5_buff[30];
	
void Task_Init(){

	  RS485Init(&rs485bus, &huart6, GPIOA, GPIO_PIN_4);// 初始化485总线管理器
	
	 //遥控器
   __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
   HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff, sizeof(usart5_buff));
   __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);




	vPortEnterCritical();
	
//	xTaskCreate(Remote,
//         "Remote",
//          400,
//          NULL,
//          3,
//          &Remote_Handle); 
					
	xTaskCreate(Hit_Task,
			 "Hit_Task",
				400,
				NULL,
				4,
				&Hit_Task_Handle); 
					
	vPortExitCritical();
	
}
