/**
 * @file    sevro.h
 * @brief   舵机驱动头文件：PWM通道定义、占空比计算宏、机械限位
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __SEVRO_H_
#define __SEVRO_H_
#include "zf_common_headfile.h"

#define SERVO_PWM1              (PWME_CH1P_PA0)
#define SERVO_PWM2              (PWME_CH2P_PA2)
#define SERVO_PWM3              (PWME_CH3P_PA4)
#define SERVO_PWM4              (PWME_CH4P_PA6)

#define SERVO_FREQ              (300)   // 舵机PWM频率，越高响应越快但力矩越小(50-300Hz)

// 舵机占空比计算:
//   0-180度对应0.5ms-2.5ms高电平脉冲
//   PWM_DUTY_MAX / (1000/freq) * (0.5 + Angle/90)
//   100Hz时90度脉宽1.5ms → PWM_DUTY_MAX/10 * 1.5
#define SERVO_DUTY(x)         ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

#if ((SERVO_FREQ < 50) || (SERVO_FREQ > 300))
    #error "SERVO_MOTOR_FREQ ERROR!"
#endif

#define SERVO_DUTY_MID   4700     // 舵机中值占空比(对应87.5度)
#define SERVO_DUTY_MIN   4200     // 舵机右转极限(机械限位)
#define SERVO_DUTY_MAX   5200     // 舵机左转极限(机械限位)
#define SERVO_CENTER     4700     // 舵机中值(别名)
#define MAX_TURN         550      // 最大有效转向量(中值±550)

#define controlReferenceLine 80
#define Mid_Col 94                // 图像中心列(188/2)，即巡线目标列

// 负压风机
extern uint8 g_fan_speed;        // 负压风机占空比(0-100)(按键/VOFA可调)

void servo_init(void);
void servo_setAngle(pwm_channel_enum sever, float angle);
void Servo_Loop(void);
void Negative_Pressure(uint32 duty);

#endif
