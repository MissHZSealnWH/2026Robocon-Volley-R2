#include "RMLibHead.h"
#include "Task_Init.h"
#include "fdcan.h"
#include "Chassis.h"
#include "FDCANDriver.h"
#include "hit_ball.h"

extern RS485_t rs485bus;
extern uint8_t usart5_buff[30];
uint8_t uart7_rx_buf[3];
extern SemaphoreHandle_t action_semaphore;

void Task_Init(){

	
	RS485Init(&rs485bus, &huart2, GPIOA, GPIO_PIN_4);// 初始化485总线管理器
	
	 //遥控器
   __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
   HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff, sizeof(usart5_buff));
   __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);

	 //H723接收F407
   HAL_UART_Receive_DMA(&huart7, uart7_rx_buf, 3);
	
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

static void Parse_Frame(uint8_t *buf)
{
	uint8_t my_data;
    // 解析接收到的3字节帧
    if (buf[0] == FRAME_HEAD && buf[2] == FRAME_TAIL)
    {
        my_data = buf[1];
        if (my_data == ACTION_CMD)
        {
					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					xSemaphoreGiveFromISR(action_semaphore, &xHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        // 可以在这里添加其他命令处理
    }
}
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{

    if(huart->Instance == UART7)
    {
        Parse_Frame(uart7_rx_buf);
			
        // 重新启动DMA
        HAL_UART_Receive_DMA(&huart7, uart7_rx_buf, 3);
    }
		
}