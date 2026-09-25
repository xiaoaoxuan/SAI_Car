/**
 * @file    sevro.c
 * @brief   舵机驱动：角度设定、PWM初始化、中值死区滤波、负压风机控制
 * @author  RRZ
 * @date    2026-05-16
 */

#include "sevro.h"

uint8 g_fan_speed = 0;   // 负压风机占空比(0-100)(按键/VOFA可调)

/**
 * @brief  设置舵机角度（相对87.5度中值±12.5度）
 */
void servo_setAngle(pwm_channel_enum sever, float angle)
{
    // 角度限幅：只允许±12.5度范围
    if (angle > 12.5 || angle < -12.5) return;

    // 转换为舵机绝对角度：中值87.5度，左转 angle 度
    angle = 87.5 - angle;

    // 计算占空比并输出
    pwm_set_duty(sever, SERVO_DUTY(angle));
}

/**
 * @brief  舵机4通道PWM初始化，SERVO_PWM1初始化为中值
 */
void servo_init(void)
{
    // 四路PWM初始化：舵机1=中值，舵机2/3/4=0（2空闲/3风机/4风机）
    pwm_init(SERVO_PWM1, SERVO_FREQ, SERVO_DUTY(87.5));
    pwm_init(SERVO_PWM2, SERVO_FREQ, 0);
    pwm_init(SERVO_PWM3, SERVO_FREQ, 0);
    pwm_init(SERVO_PWM4, SERVO_FREQ, 0);

    // 手动精确设置舵机1中值占空比为4450（消除浮点计算误差）
    pwm_set_duty(SERVO_PWM1, 4450);
}

/**
 * @brief  舵机中值死区过滤：4485~4565范围强制输出4450，消除中位抖动
 */
volatile uint16 huQiqiangServo(volatile uint16 input)
{
    // 中值死区(4485~4565) → 统一输出4450（中值）
    if ((input < 4565) && (input > 4485))
    {
        return (volatile uint16)4450;
    }
    // 死区外原样输出
    return input;
}

/**
 * @brief  舵机主循环：运行PID并输出PWM
 */
void Servo_Loop(void)
{
    // 运行舵机位置式PID，更新 Out_servo
    PID_servof(&servo_pidf);

    // 将PID计算结果输出到舵机PWM通道
    pwm_set_duty(SERVO_PWM1, (uint32)Out_servo);
}

/**
 * @brief  负压风机控制：0~100%占空比 → 50Hz PWM(1.0~1.8ms脉宽)
 * @param  duty  占空比(0-100)
 */
void Negative_Pressure(uint32 duty)
{
    uint32 PWM;

    // 占空比→PWM寄存器值：duty*24 + 3000（3000基准=100%占空/50Hz对应值）
    PWM = duty * 24 + 3000;

    // 限幅保护：1.0ms~1.8ms 脉宽对应 3000~5400
    if (PWM > 5400) PWM = 5400;   // 上限=1.8ms脉宽
    if (PWM < 3000) PWM = 3000;   // 下限=1.0ms脉宽

    // 双通道同步输出（两路风机并联）
    pwm_set_duty(SERVO_PWM3, PWM);
    pwm_set_duty(SERVO_PWM4, PWM);
}
