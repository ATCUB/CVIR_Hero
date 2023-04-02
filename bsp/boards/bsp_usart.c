#include "bsp_usart.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include "main.h"

extern UART_HandleTypeDef huart1;
uint8_t rx_buffer[100] = {0};//������
uint8_t recv_end_flag = 0;  //��ɽ��ձ�־λ
uint16_t rx_len = 0;  //�������ݳ���
uint8_t ch;
uint8_t ch_r;

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
  /* USER CODE END USART1_IRQn 0 */    
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	tmp_flag =  __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);
	if((tmp_flag != RESET))
	{			
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);

		/* ��ȡ����״̬�Ĵ��� */
		temp = huart1.Instance->SR;
		/* ��ȡ�������ݼĴ��� */
		temp = huart1.Instance->DR;
		HAL_UART_DMAStop(&huart1);
		/* ��ȡDMAʣ�ഫ������ */
		temp  = hdma_usart1_rx.Instance->NDTR;
		rx_len =  BUFFER_SIZE - temp;
		recv_end_flag = 1;
	}
  /* USER CODE END USART1_IRQn 1 */
}
/*
	printf�����ض���
*/
int fputc(int c, FILE * f)
{
	ch=c;
	HAL_UART_Transmit(&huart1,&ch,1,1000);//���ʹ���
	return c;
}

/**
  * @brief usart1DMA���ղ����ͺ�����ע�⽫���յ�rx_buffer��������.
  */
void uart1_tx_rx(void)
{
	if(recv_end_flag ==1)
	{
		HAL_UART_Transmit_DMA(&huart1,rx_buffer,rx_len);
		rx_len=0;
		recv_end_flag=0;
		HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);
	}

}
