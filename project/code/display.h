/**
 * @file    display.h
 * @brief   屏幕显示头文件：外部变量和函数声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __DISPLAY_H_
#define __DISPLAY_H_

#include "zf_common_headfile.h"

extern uint16 camera_id;    // 图像控件ID，创建时返回

extern ips200pro_image_line_uint8_struct border1[MT9V03X_H];
extern ips200pro_image_line_uint8_struct border2[MT9V03X_H];
extern uint16 rectangle_x ;
extern uint8  temp_start;
void display_init_pro(void);
void display_init(void);
void isp_200_draw_col(uint8 col ,uint8 h,uint8 start,uint16 color);
void isp_200_draw_row(uint8 col ,uint8 w,uint8 start,uint16 color);
void display_motor(void);



#endif
