/**
 * @file    Vofa_Send.h
 * @brief   VOFA通信头文件：函数声明和全局变量声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __VOFA_SEND_H_
#define __VOFA_SEND_H_

#include "zf_common_headfile.h"

extern uint8 g_motor_enable;    // 电机使能标志(VOFA命令控制)

void vofa_send_init(void);
void vofa_send_data(void);
void vofa_check_receive(void);

#endif
