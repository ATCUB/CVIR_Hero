/**
  ****************************(C) COPYRIGHT 2023 ly****************************
  * @file       message_usart.c/h
  * @brief      Here is the USART send and receive function to receive data from the upper computer, 
	*							while the UART send function sends device information
  *             这里是USART发送接收函数，接收上位机数据,UART发送函数发送设备信息.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-5-2023     ly              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 ly****************************
  */

#ifndef MESSAGE_USART_H
#define MESSAGE_USART_H
#include "struct_typedef.h"

//定义接收数组的大小
#define BUFFER_SIZE 100
extern uint8_t rx_buffer[];
extern uint8_t recv_end_flag;
extern uint16_t rx_len;

#define get_error_list_point_mgs()      get_error_list_point() //获取错误列表指针
#define MESSAGE_SEND_TIME               400


extern void UART_printf(void);
extern void msg_send_task(void const *pvParameters);



#endif
