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
	
#include "message_usart.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "detect_task.h"

#include "usart.h"

static const error_t *device_list;   //error list point

const char* device_name[ERROR_LIST_LENGHT] = {"SBUS", "motor1", "motor2", "motor3", "motor4","yaw","pitch", "trigger",
																							 "board gyro", "board accel", "board mag", "referee", "rm imu", "oled"	};


//定义接收数组的大小
extern UART_HandleTypeDef huart1;
uint8_t rx_buffer[100] = {0};				//缓冲区
uint8_t recv_end_flag = 0;  				//完成接收标志位
uint16_t rx_len = 0;  							//接收数据长度
uint8_t ch;
uint8_t ch_r;

/*
	printf函数重定向
*/
int fputc(int c, FILE * f)
{
	ch=c;
	HAL_UART_Transmit(&huart1,&ch,1,1000);//发送串口
	return c;
}

/**
  * @brief usart1DMA接收并发送函数，注意将接收的rx_buffer主动清零.
  */
void UART_printf(void)
{
	if(recv_end_flag ==1)
	{
		HAL_UART_Transmit_DMA(&huart1,rx_buffer,rx_len);
		rx_len=0;
		recv_end_flag=0;
		HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);
	}
}

/**
  * @brief message发送函数，发送设备上线信息.
  */
void msg_send_task(void const *pvParameters)
{
	static uint8_t device_status[ERROR_LIST_LENGHT] = {0};
	static uint8_t  first_call = 1;
	device_list = get_error_list_point_mgs();
	
	while(1)
	{
		for( uint8_t i=0; i < ERROR_LIST_LENGHT; i++ )
		{
			if( device_list[i].enable )
			{
				
				if( first_call )
				{
					for( uint8_t i=0; i < ERROR_LIST_LENGHT; i++ )
						device_status[i] = ( !device_list[i].is_lost );
					first_call = 0;
				}
				
				if( device_list[i].is_lost != ( !device_status[i] ) )
				{
					device_status[i] = ( !device_list[i].is_lost );
					
					if( device_list[i].is_lost == 1 )
					{
						
						printf("device name: %s is offline in  %d \r\n", device_name[i], device_list[i].lost_time);
						
					}		
					
					if( device_list[i].is_lost == 0 )
					{
						
						printf("device name: %s is online in  %d \r\n", device_name[i], device_list[i].work_time);
						
					}	
					
				}
				
			}
		}
		vTaskDelay(MESSAGE_SEND_TIME);
	}
}


