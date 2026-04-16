#include "RMLibHead.h"
#include "Task_Init.h"
#include "fdcan.h"
#include "Chassis.h"
#include "FDCANDriver.h"
#include "hit_ball.h"

extern RS485_t rs485bus;
extern uint8_t usart5_buff[30];
uint8_t rx_buf[30];
extern SemaphoreHandle_t action_semaphore;

void Task_Init(){

	
	RS485Init(&rs485bus, &huart2, GPIOA, GPIO_PIN_4);// 初始化485总线管理器
	
	 //遥控器
   __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
   HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff, sizeof(usart5_buff));
   __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);

	
	   HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buf, sizeof(rx_buf));
   __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
	
	vPortEnterCritical();
	
//	xTaskCreate(Remote,
//      	"Remote",
//        400,
//        NULL,
//        3,
//        &Remote_Handle); 
//					
	xTaskCreate(Hit_Task,
			 "Hit_Task",
				400,
				NULL,
				3,
				&Hit_Task_Handle); 
//				
//	xTaskCreate(Control_Remote,
//			 "Control_Remote",
//				312,
//				NULL,
//				3,
//				&Control_Remote_Handle); 
	
//	xTaskCreate(Ball_back,
//			 "Ball_back",
//				400,
//				NULL,
//				3,
//				&Ball_back_Handle); 
					
	vPortExitCritical();
	
}

void Parse_Frame(uint8_t *buf, uint16_t len)
{
    if(len < 3) return;

    for(uint16_t i = 0; i <= len - 3; i++)
    {
        if(buf[i]     == FRAME_HEAD &&
           buf[i + 1] == ACTION_CMD &&
           buf[i + 2] == FRAME_TAIL)
        {
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          xSemaphoreGiveFromISR(action_semaphore, &xHigherPriorityTaskWoken);
          portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
					

					break;
        }
    }
}