/**
 * @file    Motor.h
 * @brief   电机驱动头文件：引脚定义、编码器配置、变量声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

extern int16 encoder_data_r;
extern int16 encoder_data_l;
extern int16 tar_speed;
extern int16 diff_left, diff_right;

#define DIR_R              ( IO_P74 )
#define PWM_R              ( PWMB_CH2_P75 )
#define DIR_L              ( IO_P76 )
#define PWM_L              ( PWMB_CH4_P77 )

#define ENCODER_DIR_2       (PWMA_ENCODER)
#define ENCODER_DIR_PULSE_2 (PWMA_ENCODER_CH1P_P60)
#define ENCODER_DIR_DIR_2   (PWMA_ENCODER_CH2P_P62)
#define ENCODER_DIR_1       (PWMC_ENCODER)
#define ENCODER_DIR_PULSE_1 (PWMC_ENCODER_CH1P_P40)
#define ENCODER_DIR_DIR_1   (PWMC_ENCODER_CH2P_P42)

extern int16 g_speed_max;
extern int16 g_speed_min;
extern int16 g_base_speed;     // 目标基础速度(按键/VOFA可调)
extern float g_speed_ratio;    // 弯道减速比例，越小弯道越慢(VOFA可调)
extern float diff_kp;          // 差速比例系数(VOFA可调)
extern uint8 crosswalk_stop_flag;

// ---- 电机控制 ----
void Motor_Init(void);
void motor_test(void);
void Encoder_GetValue(void);
void Motor_Loop(void);
void Motor_control(pwm_channel_enum wheel, int16 speed);
void Motor_Stop(void);

#endif
