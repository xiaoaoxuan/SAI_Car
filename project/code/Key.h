/**
 * @file    Key.h
 * @brief   按键检测头文件：按键引脚定义、标志位声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __KEY_H_
#define __KEY_H_
#include "zf_common_headfile.h"

#define KEY1_PIN        IO_P91    // 按键1引脚
#define KEY2_PIN        IO_P90    // 按键2引脚
#define KEY3_PIN        IO_P67    // 按键3引脚
#define KEY4_PIN        IO_P65    // 按键4引脚

#define SWITCH1_PIN     IO_PB0    // 拨码开关1(备用)
#define SWITCH2_PIN     IO_PB1    // 拨码开关2(备用)

extern uint8 key1_flag;
extern uint8 key2_flag;
extern uint8 key3_flag;
extern uint8 key4_flag;


void Key_GetNum(void);
void Key_Anlysis(void);

#endif
