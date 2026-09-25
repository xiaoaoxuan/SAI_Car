/**
 * @file    image2.h
 * @brief   图像二值化头文件：OTSU宏定义、函数声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __IMAGE2_H_
#define __IMAGE2_H_

#include "zf_common_headfile.h"
#define GrayScale 256           // 灰度级数
#define OTSU_USE_LIMIT    0     // 1=启用OTSU阈值限幅保护，0=关闭
#define OTSU_LIMIT_MIN    90    // 限幅下限
#define OTSU_LIMIT_MAX    200   // 限幅上限

#define threshold_max	255*5   // 滤波阈值上限(备用)
#define threshold_min	255*2   // 滤波阈值下限(备用)

void base_image_threshold(uint8 Threshold);
uint8 otsuThreshold(uint8 *mt9v03x_image_first);

extern uint8 Threshold;
extern uint8 base_image[MT9V03X_H][MT9V03X_W];

#endif /* CODE_RING_H_ */
