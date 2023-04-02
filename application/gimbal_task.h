/**
  ****************************(C) COPYRIGHT 2022 LYun****************************
  * @file      	gimbal_task.c/h
  * @brief      Here is the PTZ control task, currently realizing PITCH axis motion.
  *             这里是云台控制任务，目前实现PITCH轴运动.
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


#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

#define PITCH_TURN 1
#define YAW_TURN   1
/*云台测试模式*/
#define GIMBAL_TEST_MODE 1

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201

#define GIMBAL_CONTROL_TIME 1

/*原本应定义在INS_TASK里，这里暂时定义在GIMBAL_TASK*/
#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

//turn 180°
//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F

//掉头云台速度
#define TURN_SPEED    0.04f

/*编码码盘值中值和最大值*/
#define HALF_ECD_RANGE 4096
#define ECD_RANGE      8192

//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

/*云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，*/
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

/*pitch speed close-loop PID params, max out and max iout*/
/*pitch 速度环 PID参数以及 PID最大输出，积分输出*/
#define PITCH_SPEED_PID_KP        650.0f
#define PITCH_SPEED_PID_KI        20.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

/*yaw speed close-loop PID params, max out and max iout*/
/*yaw 速度环 PID参数以及 PID最大输出，积分输出*/
#define YAW_SPEED_PID_KP        3600.0f
#define YAW_SPEED_PID_KI        20.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        20.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

/*pitch encode angle close-loop PID params, max out and max iout*/
/*pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出*/
#define PITCH_ENCODE_RELATIVE_PID_KP 1.8f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

/*yaw encode angle close-loop PID params, max out and max iout*/
/*yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出*/
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0

//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10

#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  -0.000006f //0.005

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

/*编码值转化为弧度值*/
#define MOTOR_ECD_TO_RAD    0.000766990394f

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;


extern void gimbal_task(void const *pvParameters);














#endif /*GIMBAL_TASK_H*/


