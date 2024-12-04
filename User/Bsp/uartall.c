#include "uartall.h"
#include "remote_control.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == UART5)
	{
		if (Size == RC_FRAME_LENGTH)
		{
			//HAL_UARTEx_ReceiveToIdle_IT(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2); // 接收完毕后重启
			 HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2);
			// 解析数据
			 sbus_to_rc(sbus_rx_buf, &rc_ctrl);
		}
		else  // 接收数据长度大于BUFF_SIZE，错误处理
		{	
			//HAL_UARTEx_ReceiveToIdle_IT(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2); // 接收完毕后重启
			 HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2);
			 memset(sbus_rx_buf, 0, RC_FRAME_LENGTH);							   
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
    
	if(huart->Instance == UART5)
	{
		// if (rx_cnt <= BUFF_SIZE)
		{
			HAL_UART_Receive_DMA(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2);
			// 解析数据
			sbus_to_rc(sbus_rx_buf, &rc_ctrl);
		}
		// else  // 接收数据长度大于BUFF_SIZE，错误处理	
		// {
		    
		// 	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // 接收完毕后重启
		// 	memset(rx_buff, 0, BUFF_SIZE);							   
		// }
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		//HAL_UARTEx_ReceiveToIdle_IT(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2); // 接收发生错误后重启
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2);
		 memset(sbus_rx_buf, 0, RC_FRAME_LENGTH);						   // 清除接收缓存		
	}
}
