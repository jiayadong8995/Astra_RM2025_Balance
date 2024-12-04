#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"
#include "usart.h"

// extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
// extern void RC_unable(void);
// extern void RC_restart(uint16_t dma_buf_num);
static int uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);

#endif
