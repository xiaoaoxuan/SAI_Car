/**
 * @file    main.c
 * @brief   调参愉快！
 * @author  RRZ
 * @date    2026-06-10
 */

#include "zf_common_headfile.h"
#include "Wifi_Config.h"
#include "Vofa_Send.h"
#include "tft180.h"
#include "Key.h"

uint8 ControlFlag = 0;     // 运行启停标志，0=停止 1=运行(按键4或VOFA "start"切换)
uint32 time = 0;           // 5ms计时器，系统运行时间(ms)
uint8 keynum = 0;
extern uint8 laser_flow_enable;
extern float g_ring_speed_factor;
extern float g_ring_diff_kp;
extern volatile PID_tf servo_pidf;
extern float diff_kp;
extern float g_ff_scale;

void main(void)
{
    System_Init ();
    laser_init();
    laser_off();   // 初始全部关断
//	Ring_Debug_Init();
#ifdef USE_WIFI_VOFA
    vofa_send_init();
#endif

    while (1)
    {
        if (mt9v03x_finish_flag)
        {
            // ----- 视觉处理 -----
            get_reference_point(mt9v03x_image[0]);
            Search_reference_col(mt9v03x_image[0]);
            Search_line(mt9v03x_image[0]);
            Extend_Lost_Line();
            Ring();
            Fitted_Midline();
            Error_sum();
            Error_feedforward();
//            Detect_Crosswalk();    // 斑马线检测（已注释，如需启用取消注释）
			Negative_Pressure(g_fan_speed);
            if (ring_state == 0) {
                Detect_Crossroad();
            }
            if (g_ff_scale > 5.0f) Aim_Process_MainLoop();   // 前馈缩放（g_ff_scale）>5时启动打靶，前馈已经禁用，此变量仅作为打靶启动标志位

//            Key_GetNum();
//            if (key1_flag) { g_base_speed = 100; g_fan_speed = 0; keynum = 1; key1_flag = 0; }   // 按键1: 低速100，负压关
//            if (key2_flag) { g_base_speed = 120; g_fan_speed = 0; keynum = 2; key2_flag = 0; }   // 按键2: 中速120，负压关
//            if (key3_flag) { g_base_speed = 130; g_fan_speed = 60; keynum = 3; key3_flag = 0; }  // 按键3: 高速130，负压60%
//            if (key4_flag) {
//                if (ControlFlag) {
//                    ControlFlag = 0;
//                    Ring_Init();
//                    Motor_Stop();
//                    keynum = 0;
//                } else {
//                    ControlFlag = 1;
//                    keynum = 4;
//                }
//                key4_flag = 0;
//            }

#ifdef USE_TFT
            TFT_Display();
#endif
#ifdef USE_WIFI_VOFA
            vofa_send_data();
#endif
            mt9v03x_finish_flag = 0;
        }

#ifdef USE_WIFI_VOFA
        vofa_check_receive();
#endif

//        Laser_Flow_Process();
//        laser_on();   // 9路全亮
    }
}

/**
 * @brief  5ms定时中断处理函数
 * @param  无
 * @retval 无
 * @note   每5ms执行: 陀螺仪读取→陀螺仪积分→编码器读取→打靶中断处理→
 *         启停检测(ControlFlag下降沿触发急停)→电机控制→舵机控制→计时器累加
 */
void pit_handler(void)
{
    static uint8 last_ControlFlag = 0;

    imu660ra_get_gyro();
	Gyro_Integrate();
    Encoder_GetValue();

    Aim_Process_Interrupt();

    if (last_ControlFlag == 1 && ControlFlag == 0)
    {
        Motor_Stop();
    }
    last_ControlFlag = ControlFlag;

    if (ControlFlag)
    {
        Motor_Loop();
    }

    Servo_Loop();
    time++;
}
