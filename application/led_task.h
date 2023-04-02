#ifndef LED_TASK_H
#define LED_TASK_H

#include "main.h"

#define LED_GREEN   0x7F00FF00
#define LED_YELLOW  0x7FFFFF00
#define LED_RED     0x7FFF0000
#define LED_OFF     0x00000000


extern void Gimbal_task_nowState_Show(void const * argument);
extern void Chassis_task_nowState_Show(void const * argument);














#endif /*LED_TASK_H*/
