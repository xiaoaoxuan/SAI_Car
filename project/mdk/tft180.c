/**
 * @file    tft180.c
 * @brief   TFT180屏幕显示模块：灰度图像绘制、赛道边线/中线叠加、调试文本显示
 * @author  RRZ
 * @date    2026-05-16
 */

#include "tft180.h"
#include "zf_device_tft180.h"
#include "image.h"
#include "sevro.h"
#include "Ring.h"
#include "PID.h"

extern uint8 left_control_line[IMG_H];
extern uint8 right_control_line[IMG_H];
extern uint8 mid_line[IMG_H];
extern uint8 mt9v03x_image[IMG_H][IMG_W];
extern int32 err_sum;
extern volatile uint16 Out_servo;
extern uint8 current_step;

extern float g_steer_err;

extern uint8 ring_state;
extern int32 gyro_accum;

extern uint8 valid_start_row;
extern uint8 valid_row_count;

/**
 * @brief  TFT屏幕显示主函数：灰度图+边线(黄)+中线(红)+有效区(绿)+调试文本
 * @param  无
 * @retval 无
 * @note   每帧调用一次，叠加显示视觉处理结果用于调试
 */
void TFT_Display(void)
{
    int x, y;
    uint8 i, screen_row;

    // 1. 灰度图像
    tft180_show_gray_image(
        DISP_X, DISP_Y,
        (const uint8 *)mt9v03x_image[0],
        IMG_W, IMG_H,
        DISP_W, DISP_H,        // 显示缩放尺寸
        0
    );

    // 2. 黄线=赛道边线
    for (i = 0; i < IMG_H; i++)
    {
        y = DISP_Y + i * DISP_H / IMG_H;
        if (left_control_line[i] > 0 && left_control_line[i] < IMG_W)
        {
            x = DISP_X + left_control_line[i] * DISP_W / IMG_W;
            tft180_draw_point(x, y, RGB565_YELLOW);
        }
        if (right_control_line[i] > 0 && right_control_line[i] < IMG_W)
        {
            x = DISP_X + right_control_line[i] * DISP_W / IMG_W;
            tft180_draw_point(x, y, RGB565_YELLOW);
        }
    }

    // 3. 红线=拟合中线
    for (i = 0; i < IMG_H; i++)
    {
        y = DISP_Y + i * DISP_H / IMG_H;
        x = DISP_X + mid_line[i] * DISP_W / IMG_W;
        tft180_draw_point(x, y, RGB565_RED);
    }

    // 4. 绿线=有效反馈区的起始行和结束行
    if (valid_row_count > 0)
    {
        uint8 valid_end_row = valid_start_row + valid_row_count;
        screen_row = DISP_Y + (uint16)valid_start_row * DISP_H / IMG_H;
        for (x = DISP_X; x < DISP_X + DISP_W; x++)
            tft180_draw_point(x, screen_row, RGB565_GREEN);
        screen_row = DISP_Y + (uint16)valid_end_row * DISP_H / IMG_H;
        for (x = DISP_X; x < DISP_X + DISP_W; x++)
            tft180_draw_point(x, screen_row, RGB565_GREEN);
    }

    // 5. 调试文本
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);

    // 第1行: 舵机误差
    tft180_show_string(0, 65, "Err:");
    tft180_show_int16(30, 65, (int16)g_steer_err);

    // 第2行: 圆环状态机
    tft180_show_string(0, 85, "Ring:");
    tft180_show_uint8(45, 85, ring_state);

    // 第3行: 陀螺仪积分值(圆环需要积分时)
    tft180_show_string(0, 105, "Gyro:");
    tft180_show_int32(45, 105, gyro_accum);
}
