/**
 * @file    Key.c
 * @brief   按键检测模块：4个按键的状态读取和松开标志位生成
 * @author  RRZ
 * @date    2026-05-16
 */

#include "Key.h"

// 按键状态变量
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

// 上一次按键状态变量
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

// 按键标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;

/**
 * @brief  读取按键状态并产生松开标志位
 * @param  无
 * @retval 无 (更新 keyX_flag)
 * @note   按键松开(上升沿)时对应 flag 置1，调用后需手动清零 flag
 */
void Key_GetNum(void)
{
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;

    // 获取当前按键状态
    key1_status = gpio_get_level(KEY1_PIN);
    key2_status = gpio_get_level(KEY2_PIN);
    key3_status = gpio_get_level(KEY3_PIN);
    key4_status = gpio_get_level(KEY4_PIN);

    //检测到松开按键之后  才打开按键标志位
    if(key1_status && !key1_last_status)    key1_flag = 1;
    if(key2_status && !key2_last_status)    key2_flag = 1;
    if(key3_status && !key3_last_status)    key3_flag = 1;
    if(key4_status && !key4_last_status)    key4_flag = 1;
}

/**
 * @brief  按键分析处理：读取按键并清零标志位
 * @param  无
 * @retval 无
 * @note   需在外部根据 flag 做具体处理后再调此函数清零
 */
void Key_Anlysis(void)
{
    Key_GetNum();
    if(key1_flag)
    {
        key1_flag=0;
    }
    if(key2_flag)
    {
        key2_flag=0;
    }
    if(key3_flag)
    {
        key3_flag=0;
    }
    if(key4_flag)
    {
        key4_flag=0;
    }
}
