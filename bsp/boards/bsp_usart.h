#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"

#define BUFFER_SIZE 100
extern uint8_t rx_buffer[];
extern uint8_t recv_end_flag;
extern uint16_t rx_len;

extern void uart1_tx_rx(void);
//extern void usart1_tx_dma_init(void);
//extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
#endif
