#include "Handle.h"
#include "Task_Init.h"

extern SemaphoreHandle_t action_semaphore;
extern uint8_t uart7_rx_buf[3];
extern uint8_t usart5_buff[30];
extern RS485_t rs485bus;
extern uint32_t err_timer_cnt;
extern uint32_t error_cnt;
extern uint32_t last_error_time;
extern ErrorStats_t error_stats;
extern Exp_param go_volley;
extern uint32_t err_timer_cnt;
extern int16_t can_send_buf[4];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	//遥控器
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
   		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}

		//发球宇树
	if (huart->Instance == USART2)
	{
			RS485RecvIRQ_Handler(&rs485bus, &huart2, size);
			err_timer_cnt=0;   
	}

		
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	  //遥控器
    if (huart->Instance == UART5)
    {
        HAL_UART_DMAStop(huart);
        // 重置HAL状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        
        // 然后清除错误标志 - 按照STM32H7参考手册要求的顺序
        uint32_t isrflags = READ_REG(huart->Instance->ISR);
        
        /* 清错误标志（H7用ICR） */
        if (isrflags & USART_ISR_ORE)
            huart->Instance->ICR |= USART_ICR_ORECF;

        if (isrflags & USART_ISR_FE)
            huart->Instance->ICR |= USART_ICR_FECF;

        if (isrflags & USART_ISR_NE)
            huart->Instance->ICR |= USART_ICR_NECF;

        if (isrflags & USART_ISR_PE)
            huart->Instance->ICR |= USART_ICR_PECF;
      Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff, 0);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
		
   //发球宇树
    if (huart->Instance == USART2)
    {
			
                __HAL_UART_CLEAR_FLAG(huart,
                              UART_CLEAR_OREF |
                              UART_CLEAR_FEF  |
                              UART_CLEAR_NEF  |
                              UART_CLEAR_PEF);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
         error_cnt++;
    }
	  if (huart->Instance == UART7)
    {
			__HAL_UART_DISABLE(huart);
			__HAL_UART_CLEAR_OREFLAG(huart);
			__HAL_UART_ENABLE(huart);

			// 重新启动DMA
			HAL_UART_Receive_DMA(huart, uart7_rx_buf, 3);
    }

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	  //宇树
    if (huart->Instance == USART2)
    {
        RS485SendIRQ_Handler(&rs485bus, &huart2);
    }
}

