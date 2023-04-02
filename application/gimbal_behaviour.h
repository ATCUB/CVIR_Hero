/**
  ****************************(C) COPYRIGHT 2022 LYun****************************
  * @file      	gimbal_task.c/h
  * @brief      Here is the PTZ control task, currently realizing PITCH axis motion.
  *             ��������̨��Ϊ��������Ŀǰʵ��PITCH���˶�.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-28-2022     LY              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 LYun****************************
  */

#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include "struct_typedef.h"
#include "gimbal_task.h"

/*ԭ��Ӧ������INS_TASK.H��������ʱ������GIMBAL_BEHAIVOUR.H*/
#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_ABSOLUTE_ANGLE, 
  GIMBAL_RELATIVE_ANGLE, 
  GIMBAL_MOTIONLESS,     
} gimbal_behaviour_e;

extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);




























#endif /*GIMBAL_BEHAVIOUR_H*/


