/**
 * @file    laser.h
 * @brief   激光打靶头文件：引脚定义、阈值宏、全局变量声明、函数声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __LASER_H_/* CODE_LASER_H_ */
#define __LASER_H_/* CODE_LASER_H_ */
#include "zf_common_headfile.h"

#define SHOOT_TIME  1500     // 预留射击时间(ms)，当前未使用
#define LASER_NUM   9        // 激光路数
#define LASER_PIN   IO_P01

extern uint16 tar_i;
extern uint8 tar_flag;
extern uint8 tar_detected;
extern uint8 tar_timeout;
extern uint8  tar_x;         // 靶心X坐标
extern uint8  tar_y;         // 靶心Y坐标
extern uint8 tar_th;         // 靶环灰度阈值，低于此值视为黑色靶环
extern uint8 tar_laser;      // 靶子对应激光编号 1~9
extern uint8 l_d;            // 靶子左边界
extern uint8 r_d;            // 靶子右边界
extern uint8 top_d;          // 靶子上边界
extern uint8 under_d;        // 靶子下边界
extern uint8 debug_stage;
extern uint8 pre_find_offset;    // 预扫描偏移量
extern uint8 tar_step_flags;     // bit0=下边界 bit1=上边界 bit2=左边界 bit3=右边界

// 打靶状态机变量
extern int16 g_target_col;       // 舵机目标列
extern uint8 aim_step;           // 打靶状态 0=空闲 1=点亮 2=冷却
extern uint8 shot_count;         // 射击次数

void laser_init(void);
void laser_on(void);
void laser_off(void);
void light(uint8 idx, uint8 on_off);

void Pre_Scan(void);
void Target_find(void);

// 封装好的打靶调用函数
void Aim_Process_MainLoop(void);     // 主循环中调用（每帧一次）
void Aim_Process_Interrupt(void);    // 中断中调用（每5ms一次）
void Laser_Flow_Process(void);       // 9路激光流水灯（每帧调用一次）

#endif /* CODE_LASER_H_ */
