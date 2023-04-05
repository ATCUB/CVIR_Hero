/**
  ****************************(C) COPYRIGHT 2023 ly****************************
  * @file       message_usart.c/h
  * @brief      Here is the USART send and receive function to receive data from the upper computer, 
	*							while the UART send function sends device information
  *             ������USART���ͽ��պ�����������λ������,UART���ͺ��������豸��Ϣ.
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

//�����������Ĵ�С
#define BUFFER_SIZE 100
extern uint8_t rx_buffer[];
extern uint8_t recv_end_flag;
extern uint16_t rx_len;

#define get_error_list_point_mgs()      get_error_list_point() //��ȡ�����б�ָ��
#define MESSAGE_SEND_TIME               400


extern void UART_printf(void);
extern void msg_send_task(void const *pvParameters);



#endif
