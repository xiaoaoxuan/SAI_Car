/**
 * @file    Init.c
 * @brief   系统初始化模块：时钟、IMU/编码器/舵机/电机/激光/按键/摄像头/显示屏外设初始化
 * @author  RRZ
 * @date    2026-05-16
 */

#include "Init.h"
#include "Key.h"
#include "zf_device_wifi_spi.h"

// 当 USE_TFT 时需要
#ifdef USE_TFT
    #include "display.h"
		#include "zf_device_tft180.h"
#endif

extern void pit_handler(void);

//uint8 gyro_init_ok = 0;

/**
 * @brief  系统外设初始化
 * @param  无
 * @retval 无
 * @note   初始化顺序: 时钟→调试串口→IMU→编码器→舵机→电机→激光→按键→摄像头→显示屏→5ms定时器
 *         摄像头初始化失败会重试
 */
void System_Init(void)
{
    // 系统时钟配置
    clock_init(SYSTEM_CLOCK_96M);   // 系统主频96MHz

    debug_init();

    // 陀螺仪
    imu660ra_init();
//		uint8 gyro_init_ok = (imu660ra_init() == 0) ? 1 : 0;

    // 编码器
    encoder_quad_init(ENCODER_DIR_1, ENCODER_DIR_DIR_1, ENCODER_DIR_PULSE_1);
    encoder_quad_init(ENCODER_DIR_2, ENCODER_DIR_DIR_2, ENCODER_DIR_PULSE_2);

    // 舵机
    servo_init();

    // 电机
    Motor_Init();

    // 激光（如果不需要可屏蔽对应初始化后删除）
    laser_init();

    // 按键
    gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, 1, GPI_PULL_UP);

    // 摄像头
    while (1)
    {
        if (!mt9v03x_init())
            break;
        system_delay_ms(200);   // 摄像头初始化失败重试间隔(ms)
    }

    // 通信 / 显示
//#ifdef USE_WIFI_VOFA
//    while (wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
//        system_delay_ms(100);
//    while (wifi_spi_socket_connect("TCP", PC_IP_ADDR, VOFA_PORT, LOCAL_PORT))
//        system_delay_ms(100);
//#endif

#ifdef USE_TFT
//    display_init_pro();
		tft180_init();
		tft180_clear(RGB565_BLACK);
#endif

    // 开启 5ms 定时器
    pit_ms_init(TIM_0, 5, pit_handler);   // 主中断定时器，5ms周期
}
