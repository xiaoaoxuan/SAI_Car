/**
 * @file    PID.c
 * @brief   PID控制模块：舵机位置式PID(前馈融合+陀螺仪阻尼+曲率前馈) + 电机增量式PID
 * @author  RRZ
 * @date    2026-05-16
 */

#include "PID.h"
#include "sevro.h"
#include "image.h"

volatile uint16 Out_servo = SERVO_DUTY_MID;   // 舵机PWM输出值（全局）
float g_steer_err = 0.0f;                      // 转向误差（全局，供调试/VOFA显示）

extern int16 g_target_col;

#define DEFAULT_SERVO_KP     9.5f    // 舵机比例系数：越大转向越灵敏，太大直道会抖
#define DEFAULT_SERVO_KG     0.01f   // 陀螺仪阻尼系数：越大越抑制过冲，太大会迟钝
#define DEFAULT_SERVO_KP2    0.024f  // 舵机二次项系数：大弯时额外加力

#define DEFAULT_MOTOR_KP     22.0f   // 电机比例系数：速度响应快慢
#define DEFAULT_MOTOR_KI     3.0f    // 电机积分系数：消除稳态误差
#define DEFAULT_MOTOR_KD     3.0f    // 电机微分系数：抑制超调

// 右电机PID实例
volatile PID_tf pid_rf = {
    DEFAULT_MOTOR_KP, DEFAULT_MOTOR_KI, DEFAULT_MOTOR_KD,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// 左电机PID实例
volatile PID_tf pid_lf = {
    DEFAULT_MOTOR_KP, DEFAULT_MOTOR_KI, DEFAULT_MOTOR_KD,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

// 舵机PID实例：kp/kp2/kg 已赋值，ki/kd=0（舵机用位置式，不需要积分/微分）
volatile PID_tf servo_pidf = {
    DEFAULT_SERVO_KP,
    0.0f,        // ki=0，位置式P控制
    0.0f,        // kd=0
    DEFAULT_SERVO_KG,
    DEFAULT_SERVO_KP2,
    0.0f, 0.0f, 0.0f, 0.0f
};

/**
 * @brief  浮点数限幅
 */
float limit_float(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/**
 * @brief  增量式PID：delta = kp*d_err + ki*error + kd*dd_err，用于电机速度闭环
 * @param  pid      PID结构体指针
 * @param  tar_val  目标值
 * @param  act_val  实际值
 * @retval 无 (更新 pid->out)
 */
void Increment_PIDf(volatile PID_tf *pid, float tar_val, float act_val)
{
    float error, d_err, dd_err, delta, new_out;

    // 计算当前误差及一阶/二阶差分
    error  = tar_val - act_val;                         // 当前误差
    d_err  = error - pid->err1;                         // 一阶差分（误差变化量）
    dd_err = error - 2 * pid->err1 + pid->err2;         // 二阶差分（误差变化加速度）

    // 增量式PID：delta = Kp * d_err + Ki * error + Kd * dd_err
    delta  = pid->kp * d_err;                           // 比例项：响应速度变化
    delta += pid->ki * error;                           // 积分项：消除稳态误差
    delta += pid->kd * dd_err;                          // 微分项：抑制超调

    // 累加增量 → 绝对值输出
    new_out = pid->out + delta;

    // 输出限幅保护
    pid->out = limit_float(new_out, OUT_MIN, OUT_MAX);

    // 更新误差历史
    pid->err2 = pid->err1;   // err2 ← 上上次误差
    pid->err1 = error;       // err1 ← 上次误差
}

/**
 * @brief  舵机位置式PID + 前后段动态融合前馈 + 陀螺仪阻尼 + 曲率前馈 + 误差微分
 * @param  pid  舵机PID结构体指针
 * @retval 无 (更新 Out_servo)
 * @note   环岛模式下禁用前馈仅用反馈+陀螺仪；正常模式下按方向一致性动态调整前馈权重
 */
void PID_servof(volatile PID_tf *pid)
{
    extern int32 err_sum;
    extern int32 err_feedforward;
    extern float ff_confidence;
    extern float g_ff_weight;
    extern float g_diff_threshold;
    extern int16 g_target_col;
    extern uint8 ring_disable_feedforward;
    extern int32 err_far, err_near;
    extern uint8 valid_row_count;

    #define CURVATURE_GAIN 0.3f   // 曲率前馈增益
    #define D_GAIN 0.2f           // 误差微分增益

    // 方向滤波和误差历史（静态保持迭代记忆）
    static float filtered_fb_dir = 0.0f;
    static float filtered_ff_dir = 0.0f;
    static float last_err = 0.0f;

    float feedback_err, feedforward_err;
    float feedback_dir, feedforward_dir;
    float base_ff_weight, ff_weight, fb_weight;
    float diff, conflict_ratio;
    float err;
    float p_out, kp2_out;
    float curvature, curvature_ff;
    float d_err;
    float duty;

    #define DIR_FILTER_ALPHA  1.0f   // 方向滤波系数(1=不过滤)
    #define FB_DIR_DEADZONE   2.0f   // 反馈方向死区

    // ==================== 环岛模式：仅反馈控制 ====================
    if (ring_disable_feedforward)
    {
        // 环岛强制转向（预置转角值）
        if (ring_force_steer != 0)
        {
            pid->out = (float)ring_force_steer;                 // 直接使用预设转角
            pid->out = limit_float(pid->out, OUT_MIN, OUT_MAX);
            duty = (float)SERVO_DUTY_MID - pid->out;            // 中值 - 偏移 = PWM值
            duty = limit_float(duty, (float)SERVO_DUTY_MIN, (float)SERVO_DUTY_MAX);
            Out_servo = (uint16)duty;
            return;
        }

        // 无强制转向：只用远近段加权误差做反馈控制
        g_steer_err = (float)err_sum - (float)g_target_col;    // 误差 = 中线加权和 - 目标中心列

        p_out   = pid->kp * g_steer_err;                        // P项：比例控制
        kp2_out = pid->kp2 * g_steer_err * fabs(g_steer_err);   // 二次项：大误差时额外加力（非对称，正比于误差²）
        pid->out = p_out + kp2_out + pid->kg * ((float)imu660ra_gyro_z);  // 陀螺仪阻尼：抑制角速度

        // 限幅 + 转换为PWM占空比
        pid->out = limit_float(pid->out, OUT_MIN, OUT_MAX);
        duty = (float)SERVO_DUTY_MID - pid->out;
        duty = limit_float(duty, (float)SERVO_DUTY_MIN, (float)SERVO_DUTY_MAX);
        Out_servo = (uint16)duty;
        return;
    }

    // ==================== 正常模式：反馈 + 前馈融合 ====================

    // ---- 1. 计算反馈误差和前馈误差 ----
    feedback_err    = (float)err_sum - (float)g_target_col;        // 反馈误差：基于近场(底部)中线
    feedforward_err = (float)err_feedforward - (float)g_target_col; // 前馈误差：基于远场(顶部)中线

    // ---- 2. 判断反馈方向与前馈方向 ----
    // 用多行均值判断方向，减少单点噪声干扰
    feedback_dir    = ((float)mid_line[88] + (float)mid_line[90] + (float)mid_line[92]) / 3.0f - (float)g_target_col;
    feedforward_dir = ((float)mid_line[23] + (float)mid_line[25] + (float)mid_line[27]) / 3.0f - (float)g_target_col;

    // 一阶低通滤波平滑方向信号
    filtered_fb_dir = DIR_FILTER_ALPHA * feedback_dir + (1.0f - DIR_FILTER_ALPHA) * filtered_fb_dir;
    filtered_ff_dir = DIR_FILTER_ALPHA * feedforward_dir + (1.0f - DIR_FILTER_ALPHA) * filtered_ff_dir;

    // ---- 3. 根据方向一致性动态调整前馈权重 ----
    if (fabs(filtered_fb_dir) < FB_DIR_DEADZONE)
    {
        // 反馈方向在死区内（近直线）：使用全量前馈权重
        base_ff_weight = g_ff_weight;
    }
    else if (filtered_fb_dir * filtered_ff_dir < 0)
    {
        // 前后方向相反（前馈不可靠）：大幅降低前馈权重到0.05
        base_ff_weight = 0.05f;
    }
    else
    {
        // 方向一致但可能量值冲突：按差值比例衰减前馈权重
        diff = fabs(feedback_err - feedforward_err);              // 远近误差差异
        conflict_ratio = diff / g_diff_threshold;                 // 冲突比例(0~1)
        if (conflict_ratio > 1.0f) conflict_ratio = 1.0f;
        base_ff_weight = g_ff_weight * (1.0f - conflict_ratio);  // 差异越大权重越低
        if (base_ff_weight < 0.05f) base_ff_weight = 0.05f;      // 保底5%
    }

    // ---- 4. 最终前馈权重 = 基础权重 × 置信度 ----
    ff_weight = base_ff_weight * ff_confidence;

    // ---- 5. 反馈权重：保证主导地位(70%~100%) ----
    fb_weight = 1.0f - ff_weight + 0.4f;
    if (fb_weight < 0.7f) fb_weight = 0.7f;   // 最低70%反馈权重
    if (fb_weight > 1.0f) fb_weight = 1.0f;   // 最高100%

    // ---- 6. 加权融合误差 ----
    err = fb_weight * feedback_err + ff_weight * feedforward_err;
    g_steer_err = err;

    // ---- 7. 曲率前馈：远场和近场测得的弯道曲率 ----
    curvature = (float)(err_far - err_near);                      // 曲率 = 远区误差 - 近区误差
    if (curvature < -5.0f || curvature > 5.0f)
        curvature_ff = curvature * CURVATURE_GAIN;                // 曲率>5才启用，防止直线误触发
    else
        curvature_ff = 0.0f;

    // ---- 8. PID计算 ----
    p_out   = pid->kp * err;                                      // P项：比例控制
    kp2_out = pid->kp2 * err * fabs(err);                         // 二次项：大误差额外加力
    pid->out = p_out + kp2_out + pid->kg * ((float)imu660ra_gyro_z) + curvature_ff;
                                                                    //  kg项：陀螺仪角速度阻尼，抑制过冲
                                                                    //  curvature_ff：弯道曲率前馈，提前预判

    // ---- 9. 误差微分项：抑制震荡 ----
    d_err = err - last_err;                                       // 误差变化率
    last_err = err;
    pid->out += D_GAIN * d_err;                                   // D项：微分抑制

    // ---- 10. 输出限幅 + 转换为PWM占空比 ----
    pid->out = limit_float(pid->out, OUT_MIN, OUT_MAX);
    duty = (float)SERVO_DUTY_MID - pid->out;                     // 中值PWM - 转向偏移 = 实际PWM
    duty = limit_float(duty, (float)SERVO_DUTY_MIN, (float)SERVO_DUTY_MAX);  // 机械限位保护
    Out_servo = (uint16)duty;
}
