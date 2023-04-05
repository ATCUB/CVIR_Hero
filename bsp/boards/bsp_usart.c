#include "bsp_usart.h"
#include "usart.h"
#include "message_usart.h"

extern UART_HandleTypeDef huart1;

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

		/* ¶ÁÈ¡´®¿Ú×´Ì¬¼Ä´æÆ÷ */
		temp = huart1.Instance->SR;
		/* ¶ÁÈ¡´®¿ÚÊý¾Ý¼Ä´æÆ÷ */
		temp = huart1.Instance->DR;
		HAL_UART_DMAStop(&huart1);
		/* ¶ÁÈ¡DMAÊ£Óà´«ÊäÊýÁ¿ */
		temp  = hdma_usart1_rx.Instance->NDTR;
		rx_len =  BUFFER_SIZE - temp;
		recv_end_flag = 1;
	}
  /* USER CODE END USART1_IRQn 1 */
}
