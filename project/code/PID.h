/**
 * @file    PID.h
 * @brief   PID控制器头文件：PID结构体定义、输出限幅、外部变量声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __PID_H_
#define __PID_H_

#include "zf_common_headfile.h"

#define OUT_MAX     3000    // PID输出上限(对应PWM占空比)
#define OUT_MIN    -3000    // PID输出下限(对应PWM占空比)

extern volatile uint16 Out_servo;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kg;       // 陀螺仪阻尼系数
    float kp2;      // 二次项系数（大弯加力）
    float kf;
    float err1;
    float err2;
    float err_sum;
    float out;
} PID_tf;

extern volatile PID_tf pid_rf;
extern volatile PID_tf pid_lf;
extern volatile PID_tf servo_pidf;

void PID_servof(volatile PID_tf *pid);
void Increment_PIDf(volatile PID_tf *pid, float tar_val, float act_val);

#endif
