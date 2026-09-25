/**
 * @file    Ring.h
 * @brief   圆环/环岛检测头文件：状态变量声明、函数声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __RING_H_
#define __RING_H_
#include "zf_common_headfile.h"

extern uint8 ring_state;               // 圆环状态机: 0=非环,1~5=入环→出环各阶段
extern uint8 ring_scan_dir;
extern uint8 ring_l, ring_r;
extern uint8 ring_compensate;
extern uint8 upper_turn_row;
extern uint8 upper_base_col;
extern uint8 low_turn_confirmed;
extern int32 gyro_accum;               // 陀螺仪积分值（32位防止溢出）
extern uint8 debug_comp_row;
extern uint8 debug_comp_start_col;
extern uint8 debug_ring_dir;
extern uint8 ring_disable_feedforward;  // 环岛内禁用前馈标志
extern uint8 ring_exit_compensate;
extern uint8 exit_turn_row;
extern uint8 exit_turn_col;
extern int16 ring_force_steer;          // 环岛强制转向值
extern uint8 low_turn_row;
extern uint8 low_turn_col;
extern uint8 ring_comp_line[];

// ---- 圆环状态机 ----
void Ring(void);
void Ring_Init(void);
void Gyro_Integrate(void);
uint8 isContinueLine(uint8* arr, uint8 start_row, uint8 end_row);
uint8 IsLineStraight(uint8 *arr, uint8 start, uint8 end);
uint16 Count_Lost_Lines(uint8 is_left);
#endif
