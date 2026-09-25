/**
 * @file    tft180.h
 * @brief   TFT180屏幕显示头文件：分辨率宏定义、函数声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __TFT180_H_
#define __TFT180_H_

#include "zf_common_headfile.h"

// TFT屏幕实际分辨率
#define TFT_WIDTH   128
#define TFT_HEIGHT  160

// 图像显示区域（从188x120缩放至94x60）
#define DISP_W      94        // 图像显示缩放宽度
#define DISP_H      60        // 图像显示缩放高度

// 显示区域在屏幕上的偏移
#define DISP_X      0         // 显示起始X偏移
#define DISP_Y      0         // 显示起始Y偏移

// 图像原始分辨率
#define IMG_W   MT9V03X_W     // 188
#define IMG_H   MT9V03X_H     // 120

void TFT_Display(void);

#endif
