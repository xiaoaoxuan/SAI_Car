/**
 * @file    Motor.c
 * @brief   电机驱动与控制：PWM初始化、差分速度决策、增量PID闭环、低通滤波输出、急停
 * @author  RRZ
 * @date    2026-05-16
 */

#include "Motor.h"
#include "sevro.h"
#include "PID.h"
#include "image.h"

int16 encoder_data_r = 0;
int16 encoder_data_l = 0;
int16 tar_speed = 0;
int16 diff_left = 0, diff_right = 0;
extern float g_steer_err;

int16 g_base_speed = 110;            // 目标基础速度(按键/VOFA可调)
float g_speed_ratio = 0.65f;         // 弯道减速比例，越小弯道越慢(VOFA可调)
float diff_kp = 0.55f;               // 差速比例系数(VOFA可调)

float g_ring_speed_factor = 0.80f;   // 圆环降速系数(按键可调)
float g_ring_diff_kp = 0.7f;         // 入环差速KP(临时替换diff_kp)

/**
 * @brief  电机GPIO和PWM初始化
 */
void Motor_Init(void)
{
    // 左电机：方向引脚 + PWM通道，初始方向=前进
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_L, 17000, 0);   // 17kHz PWM，初始占空比0

    // 右电机：方向引脚 + PWM通道，初始方向=前进
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_R, 17000, 0);
}

/**
 * @brief  电机简易自检：左右轮各正反转1秒
 */
void motor_test(void)
{
    // 左轮正转1秒
    gpio_set_level(DIR_L, GPIO_HIGH);
    pwm_set_duty(PWM_L, 1000);
    system_delay_ms(1000);

    // 左轮反转1秒
    gpio_set_level(DIR_L, GPIO_LOW);
    system_delay_ms(1000);
    pwm_set_duty(PWM_L, 0);

    // 右轮正转1秒
    gpio_set_level(DIR_R, GPIO_HIGH);
    pwm_set_duty(PWM_R, 1000);
    system_delay_ms(1000);

    // 右轮反转1秒
    gpio_set_level(DIR_R, GPIO_LOW);
    system_delay_ms(1000);
    pwm_set_duty(PWM_R, 0);
}

/**
 * @brief  单轮差分驱动：正=前进, 负=后退
 * @param  wheel  PWM_L / PWM_R
 * @param  speed  速度值
 */
void Motor_control(pwm_channel_enum wheel, int16 speed)
{
    if (speed >= 0)
    {
        // 正转：方向引脚拉低，输出正向PWM
        if (wheel == PWM_L)
        {
            gpio_set_level(DIR_L, GPIO_LOW);
            pwm_set_duty(PWM_L, speed);
        }
        else
        {
            gpio_set_level(DIR_R, GPIO_LOW);
            pwm_set_duty(PWM_R, speed);
        }
    }
    else
    {
        // 反转：方向引脚拉高，PWM取绝对值
        if (wheel == PWM_L)
        {
            gpio_set_level(DIR_L, GPIO_HIGH);
            pwm_set_duty(PWM_L, -speed);
        }
        else
        {
            gpio_set_level(DIR_R, GPIO_HIGH);
            pwm_set_duty(PWM_R, -speed);
        }
    }
}

/**
 * @brief  读取编码器值并清零（每5ms中断调用）
 */
void Encoder_GetValue(void)
{
    // 右编码器取反（机械安装方向），读取后清零
    encoder_data_r = -encoder_get_count(ENCODER_DIR_1);
    // 左编码器直接读取
    encoder_data_l = encoder_get_count(ENCODER_DIR_2);

    // 清零编码器计数，准备下一个5ms周期
    encoder_clear_count(ENCODER_DIR_1);
    encoder_clear_count(ENCODER_DIR_2);
}

/**
 * @brief  速度决策：弯道线性减速 + 差速 + 圆环降速 + 有效行不足减速 + 低通滤波
 */
void Dream_speed(void)
{
    uint16 tp_turn;
    int8 delta = (int8)g_steer_err;   // 转向误差转为8位有符号

    // 低通滤波状态变量（静态保持滤波记忆）
    static float diff_left_smooth = 0.0f;
    static float diff_right_smooth = 0.0f;
    #define DIFF_SMOOTH_ALPHA 0.5f   // 差速低通滤波系数

    // ---- 1. 计算差速值（内外轮速度差）----
    if (Out_servo > SERVO_CENTER)
    {
        // 左转：左轮减速、右轮加速
        tp_turn = Out_servo - SERVO_CENTER;
        diff_left  = (int16)(0.76f * delta * diff_kp);   // 左轮受0.76衰减
        diff_right = (int16)(-delta * diff_kp);           // 右轮全量
    }
    else
    {
        // 右转：右轮减速、左轮加速
        tp_turn = SERVO_CENTER - Out_servo;
        delta = -delta;
        diff_left  = (int16)(-delta * diff_kp);
        diff_right = (int16)(0.76f * delta * diff_kp);   // 右轮受0.76衰减
    }

    // 限幅：防止转向量超最大值
    if (tp_turn > MAX_TURN) tp_turn = MAX_TURN;

    // ---- 2. 弯道线性减速：转向越大，速度越低 ----
    tar_speed = g_base_speed - (uint32)(g_base_speed * (1.0f - g_speed_ratio)) * tp_turn / MAX_TURN;

    // ---- 3. 圆环降速：入环阶段额外降低目标速度 ----
    {
        extern uint8 ring_state;
        extern float g_ring_speed_factor;
        if (ring_state == 1 || ring_state == 2 || ring_state == 3 || ring_state == 4) {
            tar_speed = (int16)((float)tar_speed * g_ring_speed_factor);
        }
    }

    // ---- 4. 有效行不足时提前减速（视野变短）----
    {
        extern uint8 valid_row_count;
        uint8 missing = 106 - valid_row_count;       // 缺失的有效行数
        int16 preview = (int16)missing * 20 / 106;    // 按比例换算减速量
        tar_speed -= preview;
        if (tar_speed < 40) tar_speed = 40;            // 最低速度保护
    }

    // ---- 5. 差速低通滤波：平滑差速变化，减少突变抖动 ----
    diff_left_smooth  = DIFF_SMOOTH_ALPHA * diff_left  + (1.0f - DIFF_SMOOTH_ALPHA) * diff_left_smooth;
    diff_right_smooth = DIFF_SMOOTH_ALPHA * diff_right + (1.0f - DIFF_SMOOTH_ALPHA) * diff_right_smooth;

    diff_left  = (int16)diff_left_smooth;
    diff_right = (int16)diff_right_smooth;
}

/**
 * @brief  电机主循环：速度决策→增量PID→输出滤波→驱动
 * @note   斑马线停车时跳过PID直接输出0
 */
void Motor_Loop(void)
{
    // 输出低通滤波状态变量
    static float motor_left_out_filtered  = 0.0f;
    static float motor_right_out_filtered = 0.0f;
    #define MOTOR_FILTER_ALPHA 0.7f   // 电机输出低通滤波系数

    // 斑马线停车：直接切断电机输出
    if (crosswalk_stop_flag)
    {
        Motor_control(PWM_L, 0);
        Motor_control(PWM_R, 0);
        return;
    }

    // ---- 1. 速度决策：计算目标速度 + 差速 ----
    Dream_speed();

    // ---- 2. 增量PID闭环：根据目标速度和编码器反馈计算输出 ----
    Increment_PIDf(&pid_lf, (float)(tar_speed + diff_left),  encoder_data_l);
    Increment_PIDf(&pid_rf, (float)(tar_speed + diff_right), encoder_data_r);

    // ---- 3. 一阶低通滤波：平滑PID输出，抑制高频抖动 ----
    motor_left_out_filtered  = MOTOR_FILTER_ALPHA * pid_lf.out
                             + (1.0f - MOTOR_FILTER_ALPHA) * motor_left_out_filtered;
    motor_right_out_filtered = MOTOR_FILTER_ALPHA * pid_rf.out
                             + (1.0f - MOTOR_FILTER_ALPHA) * motor_right_out_filtered;

    // ---- 4. 输出驱动 ----
    Motor_control(PWM_L, (int16)motor_left_out_filtered);
    Motor_control(PWM_R, (int16)motor_right_out_filtered);
}

/**
 * @brief  急停：切断电机输出并清零PID状态
 */
void Motor_Stop(void)
{
    // 切断电机输出
    Motor_control(PWM_L, 0);
    Motor_control(PWM_R, 0);

    // 清零PID误差历史（防止重启后积分残留抖动）
    pid_lf.err1 = 0.0f;
    pid_lf.err2 = 0.0f;
    pid_lf.out  = 0.0f;
    pid_rf.err1 = 0.0f;
    pid_rf.err2 = 0.0f;
    pid_rf.out  = 0.0f;

    // 清零速度相关变量
    tar_speed = 0;
    diff_left  = 0;
    diff_right = 0;

    // 清除停车标志
    crosswalk_stop_flag = 0;
}
