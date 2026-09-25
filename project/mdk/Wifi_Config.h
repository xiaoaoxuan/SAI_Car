/**
 * @file    Wifi_Config.h
 * @brief   WiFi配置头文件：WiFi SSID/密码、上位机IP/端口、编译选项开关
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __WIFI_CONFIG_H_
#define __WIFI_CONFIG_H_

#define USE_WIFI_VOFA                    // 启用WiFi VOFA通信（与 USE_TFT 互斥）

#ifndef USE_WIFI_VOFA
//    #define USE_TFT                    // 启用TFT屏幕显示（与 VOFA 互斥）
#endif

#ifdef USE_WIFI_VOFA
//    #define WIFI_SSID       "Redmi K70"
//    #define WIFI_PASSWORD   "njlz20060828"
//    #define PC_IP_ADDR      "10.96.250.226"   // 备用上位机IP
    #define WIFI_SSID       "RRZ"             // WiFi热点名称
    #define WIFI_PASSWORD   "123698745"        // WiFi密码
    #define PC_IP_ADDR      "10.249.228.226"   // 上位机IP地址
    #define VOFA_PORT       "8080"             // VOFA目标端口
    #define LOCAL_PORT      "0"                // 本地端口，0=自动分配
#endif

#define VOFA_SEND_FRAME_DIV   2               // VOFA发送分频，每2帧发1次

#endif
