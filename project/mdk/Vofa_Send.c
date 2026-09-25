/**
 * @file    Vofa_Send.c
 * @brief   VOFA+上位机通信模块：WiFi SPI数据发送(20通道)、命令接收解析
 * @author  RRZ
 * @date    2026-05-16
 */

#include "vofa_send.h"
#include "Wifi_Config.h"
#include "zf_device_wifi_spi.h"
#include "zf_common_headfile.h"

#include "sevro.h"
#include "Motor.h"
#include "image.h"
#include "PID.h"
#include "Ring.h"

// ==================== 外部变量引用 ====================
extern volatile uint16 Out_servo;
extern volatile PID_tf servo_pidf;
extern volatile PID_tf pid_lf, pid_rf;
extern int16 tar_speed;
extern int16 encoder_data_l, encoder_data_r;
extern int16 diff_left, diff_right;
extern int32 err_sum, err_feedforward;
extern float diff_kp;                 // 差速比例系数(VOFA可调)
extern uint8 ControlFlag;
extern int16 imu660ra_gyro_z;

extern float g_ff_scale;             // 前馈缩放系数(VOFA可调)
extern float g_ff_weight;            // 前馈融合权重(VOFA可调)
extern float g_diff_threshold;       // 远近误差差异阈值(VOFA可调)
extern int16 g_base_speed;           // 目标基础速度(按键/VOFA可调)
extern uint8 g_fan_speed;            // 负压风机占空比(0-100)(按键/VOFA可调)
extern uint8 ring_state;

// ==================== 全局变量 ====================
uint8 g_motor_enable = 0;

// ==================== 静态变量 ====================
static uint8 inited = 0;
static uint8 send_div = 0;
static char tx_buffer[256];
static char rx_buffer[64];
static uint8 rx_index = 0;

static void parse_command(const char *cmd);

/**
 * @brief  VOFA WiFi SPI初始化：连接WiFi并建立TCP连接
 * @param  无
 * @retval 无
 * @note   仅在 USE_WIFI_VOFA 宏定义时生效
 */
void vofa_send_init(void)
{
#ifdef USE_WIFI_VOFA
    if (!inited)
    {
        wifi_spi_init(WIFI_SSID, WIFI_PASSWORD);
        system_delay_ms(500);
        wifi_spi_socket_connect("TCP", PC_IP_ADDR, VOFA_PORT, LOCAL_PORT);
        inited = 1;
    }
#endif
}

/**
 * @brief  发送数据到VOFA上位机（20个数据通道）
 * @param  无
 * @retval 无
 * @note   按 VOFA_SEND_FRAME_DIV 分频发送，每帧格式化20个通道的CSV数据
 *         I0~I4: 图像误差、舵机输出、舵机KP/KG/KP2
 *         I5~I7: 电机KP/KI/KD
 *         I8: 基础速度
 *         I9~I11: 前馈参数(缩放/权重/阈值)
 *         I12~I15: 左右目标速度/实际速度
 *         I16: 陀螺仪角速度
 *         I17: 差速系数
 *         I18: 环岛状态(×100放大)
 *         I19: 风扇占空比
 */
void vofa_send_data(void)
{
    char *ptr;
    int len;
    float img_err, steer_out, steer_p, steer_kg, steer_kp2;
    float motor_kp, motor_ki, motor_kd;
    float base_speed;
    float ff_scale, ff_weight, diff_thres;
    float left_target, left_actual, right_target, right_actual;
    float gyro_z;
    float diff_kp_val;

    if (!inited) return;

    send_div++;
    if (send_div < VOFA_SEND_FRAME_DIV) return;   // VOFA发送分频，每N帧发1次
    send_div = 0;

    img_err   = (float)(err_sum - Mid_Col);
    steer_out = (float)Out_servo - (float)SERVO_DUTY_MID;
    steer_p   = servo_pidf.kp;
    steer_kg  = servo_pidf.kg;
    steer_kp2 = servo_pidf.kp2;

    motor_kp = pid_lf.kp;
    motor_ki = pid_lf.ki;
    motor_kd = pid_lf.kd;

    base_speed = (float)g_base_speed;

    ff_scale  = g_ff_scale;
    ff_weight = g_ff_weight;
    diff_thres = g_diff_threshold;

    left_target = (float)(tar_speed + diff_left);
    left_actual = (float)encoder_data_l;
    right_target = (float)(tar_speed + diff_right);
    right_actual = (float)encoder_data_r;

    gyro_z = (float)imu660ra_gyro_z;
    diff_kp_val = diff_kp;

    ptr = tx_buffer;

    // I0 ~ I4（图像误差、舵机输出、舵机PID）
    len = sprintf(ptr, "%.1f,%.1f,%.2f,%.3f,%.5f,",
                  img_err, steer_out, steer_p, steer_kg, steer_kp2);
    ptr += len;

    // I5 ~ I7（电机PID）
    len = sprintf(ptr, "%.3f,%.3f,%.3f,", motor_kp, motor_ki, motor_kd);
    ptr += len;

    // I8（基础速度）
    len = sprintf(ptr, "%.1f,", base_speed);
    ptr += len;

    // I9 ~ I15（前馈3个 + 电机4个）
    len = sprintf(ptr, "%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,",
                  ff_scale, ff_weight, diff_thres,
                  left_target, left_actual, right_target, right_actual);
    ptr += len;

    // I16（陀螺仪角速度）
    len = sprintf(ptr, "%.1f,", gyro_z);
    ptr += len;

    // I17（差速系数 diff_kp）
     len = sprintf(ptr, "%.2f,", diff_kp_val);
    ptr += len;

    // I18（环岛状态）
    len = sprintf(ptr, "%d,", ring_state * 100);
    ptr += len;

    // I19（风扇占空比百分比）
    sprintf(ptr, "%d\n", g_fan_speed);

    wifi_spi_send_string(tx_buffer);
}

/**
 * @brief  检查并解析WiFi接收缓冲区中的VOFA命令
 * @param  无
 * @retval 无
 * @note   以'!'为命令结束符，逐字节接收，完整命令后调用 parse_command()
 */
void vofa_check_receive(void)
{
    uint8 byte;
    uint32 len;

    if (!inited) return;

    while (1)
    {
        len = wifi_spi_read_buffer(&byte, 1);
        if (len == 0) break;

        if (byte == '!')
        {
            rx_buffer[rx_index] = '\0';
            parse_command(rx_buffer);
            rx_index = 0;
        }
        else
        {
            if (rx_index < sizeof(rx_buffer) - 1)
                rx_buffer[rx_index++] = byte;
        }
    }
}

/**
 * @brief  解析VOFA命令并执行对应操作
 * @param  cmd  以\0结尾的命令字符串
 * @retval 无
 * @note   支持实时调节PID参数、速度、前馈参数、风扇、启停
 *         命令格式: 参数名=浮点值! 或 start!/stop!
 */
static void parse_command(const char *cmd)
{
    float val;

    if (sscanf(cmd, "kp=%f", &val) == 1) {          // 舵机KP
        servo_pidf.kp = val;
    }
    else if (sscanf(cmd, "kg=%f", &val) == 1) {     // 陀螺仪阻尼系数
        servo_pidf.kg = val;
    }
    else if (sscanf(cmd, "kp_two=%f", &val) == 1) { // 舵机二次项系数
        servo_pidf.kp2 = val;
    }
    else if (sscanf(cmd, "motor_kp=%f", &val) == 1) { // 电机KP(左右同步)
        pid_lf.kp = val;
        pid_rf.kp = val;
    }
    else if (sscanf(cmd, "motor_ki=%f", &val) == 1) { // 电机KI(左右同步)
        pid_lf.ki = val;
        pid_rf.ki = val;
    }
    else if (sscanf(cmd, "motor_kd=%f", &val) == 1) { // 电机KD(左右同步)
        pid_lf.kd = val;
        pid_rf.kd = val;
    }
    else if (sscanf(cmd, "ff_scale=%f", &val) == 1) { // 前馈缩放系数
        g_ff_scale = val;
    }
    else if (sscanf(cmd, "ff_weight=%f", &val) == 1) { // 前馈融合权重
        g_ff_weight = val;
    }
    else if (sscanf(cmd, "diff_thres=%f", &val) == 1) { // 远近误差差异阈值
        g_diff_threshold = val;
    }
    else if (sscanf(cmd, "base_speed=%f", &val) == 1) { // 基础速度
        g_base_speed = (int16)val;
    }
    else if (sscanf(cmd, "diff_kp=%f", &val) == 1) { // 差速比例系数
        diff_kp = val;
    }
	else if (sscanf(cmd, "fan=%f", &val) == 1) {     // 风扇占空比(0-100)
		if (val < 0) val = 0;
		if (val > 100) val = 100;
		g_fan_speed = val;
		Negative_Pressure(g_fan_speed);
	}
    else if (strcmp(cmd, "start") == 0) {            // 启动
        g_motor_enable = 1;
        ControlFlag = 1;
    }
    else if (strcmp(cmd, "stop") == 0) {             // 停止
        g_motor_enable = 0;
        ControlFlag = 0;
        Ring_Init();
        Motor_Stop();
    }
}
