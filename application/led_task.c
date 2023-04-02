
#include "led_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"


///**
//  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
//  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
//  * @retval         none
//  */
///**
//  * @brief          LED初始化
//  * @param[out]     .
//  * @retval         none
//  */
//static void LED_Show_Init(void)
//{
//	return;
//}

/**
  * @brief          
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          显示云台任务状态，LED：绿色——正常， 黄色——校准中， 红色——异常中
  * @param[out]     .
  * @retval         none
  */
void Gimbal_task_nowState_Show(void const * argument)
{
	while(1)
	{	
		aRGB_led_show(LED_YELLOW);
		osDelay(500);
		aRGB_led_show(LED_GREEN);
		osDelay(500);
	}
}

/**
  * @brief          
  * @param[out]     gimbal_set_mode: "chassis_control" valiable point
  * @retval         none
  */
/**
  * @brief          显示底盘任务状态，LED：绿色——正常， 黄色——校准中， 红色——异常中
  * @param[out]     .
  * @retval         none
  */
void Chassis_task_nowState_Show(void const * argument)
{
	while(1)
	{
		aRGB_led_show(LED_YELLOW);
		osDelay(500);
		aRGB_led_show(LED_GREEN);
		osDelay(500);
	}
}





