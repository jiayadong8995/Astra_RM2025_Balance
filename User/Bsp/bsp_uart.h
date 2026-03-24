#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

void uart1_tx_dma_transmit(uint8_t *buff,uint16_t len);



#endif // !UARTALL
