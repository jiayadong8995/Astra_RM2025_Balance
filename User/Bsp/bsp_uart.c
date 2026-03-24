#include "bsp_uart.h"
#include "remote_control.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

void uart1_tx_dma_transmit(uint8_t *buff,uint16_t len)
{
	if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY)
	{
		return;
	}
	
	HAL_UART_Transmit_DMA(&huart1,buff,len);
//	if(__HAL_DMA_GET_FLAG(&hdma_usart1_tx,DMA_FLAG_TCIF3_7))//等待DMA1通道4传输完成
//	{
//		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx,DMA_FLAG_TCIF3_7);//清除DMA1通道4传输完成标志
//		HAL_UART_DMAStop(&huart1);      //传输完成以后关闭串口DMA
//	}	
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == UART5)
	{
		if (Size == RC_FRAME_LENGTH)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2);
			
			// 解析数据
			sbus_to_rc(sbus_rx_buf, &rc_ctrl);
		}
		else  // 接收数据长度大于BUFF_SIZE，错误处理
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2);
			memset(sbus_rx_buf, 0, RC_FRAME_LENGTH);							   
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
    
	if(huart->Instance == UART5)
	{
		HAL_UART_Receive_DMA(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2);
		// 解析数据
		sbus_to_rc(sbus_rx_buf, &rc_ctrl);

	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, RC_FRAME_LENGTH*2);
		 memset(sbus_rx_buf, 0, RC_FRAME_LENGTH);						   // 清除接收缓存		
	}
}





//void uart1_tx_dma_enable(uint8_t *data, uint16_t len)
//{
//    //disable DMA
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_usart1_tx);

//    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_tx);
//    }

//    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF6);

//    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
//    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

//    __HAL_DMA_ENABLE(&hdma_usart1_tx);
//}
