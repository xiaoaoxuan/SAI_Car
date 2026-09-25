/**
 * @file    display.c
 * @brief   屏幕显示模块：IPS200/IPS200Pro/TFT180初始化、绘图辅助、调试信息显示
 * @author  RRZ
 * @date    2026-05-16
 */

#include "display.h"

uint32 ips200pro_stack_size;

// 页面创建后返回的页ID，当需要对页面进行操作时，需要传入对应的ID
uint16 page_id[3];

uint16 camera_id;

ips200pro_image_line_uint8_struct border1[MT9V03X_H];
ips200pro_image_line_uint8_struct border2[MT9V03X_H];
uint8 i = 0;

uint16 rectangle_x = 0;
uint8  temp_start = 10;


/**
 * @brief  IPS200Pro屏幕高级初始化：创建页面、设置颜色主题、创建图像控件
 * @param  无
 * @retval 无
 * @note   设置第一页标题"智能车比赛"、背景色、前景色
 */
void display_init_pro(void)
{

    // 初始化屏幕并新建一个页面
    page_id[0] = ips200pro_init("智能车比赛", IPS200PRO_TITLE_BOTTOM, 30);

    // 获取屏幕信息并赋值给相关变量 1064是屏幕接口没有焊接芯片时miso引脚无法使用获取的功能
    ips200pro_get_information(&ips200pro_information);
    ips200pro_get_free_stack_size(&ips200pro_stack_size);
    ips200pro_get_time(&ips200pro_time);

    // 设置全局字体为16号
    ips200pro_set_default_font(FONT_SIZE_16);

    // 屏幕方向设置为竖屏
    ips200pro_set_direction(IPS200PRO_PORTRAIT);

    // 设置编码格式
    ips200pro_set_format(IPS200PRO_FORMAT_GBK);

    // 设置选中页面的标题背景颜色，需要写入第一个参数生成的页面ID等等
    ips200pro_set_color(page_id[0], COLOR_PAGE_SELECTED_BG, IPS200PRO_RGB888_TO_RGB565(0xFF, 0xE3, 0xE3));

    // 设置第一个页面的背景色
    ips200pro_set_color(page_id[0], COLOR_BACKGROUND, IPS200PRO_RGB888_TO_RGB565(0x43, 0x4A, 0x61));

    // 设置页面文字的背景颜色，需要写入第一个参数生成的页面ID等等
    ips200pro_set_color(page_id[0], COLOR_FOREGROUND, RGB565_BLACK);

    // 设置页面选中后的文字背景颜色，需要写入第一个参数生成的页面ID等等
    ips200pro_set_color(page_id[0], COLOR_PAGE_SELECTED_TEXT, RGB565_WHITE);

    // 设置屏幕的背光亮度，有效设置范围0-255
    ips200pro_set_backlight(255);   // 背光亮度(0-255)，降低可省电

    // 切换页面，切换到第一个页面
    ips200pro_page_switch(page_id[0] ,PAGE_ANIM_ON);

    // 创建一个图像控件，图像控件没有任何的默认颜色
    camera_id = ips200pro_image_create(26, 20, MT9V03X_W, MT9V03X_H);   // 图像在屏幕上的显示起始坐标(26,20)

}

/**
 * @brief  IPS200基础初始化（非Pro版本）
 * @param  无
 * @retval 无
 */
void display_init(void)
{
    ips200_init();
}

/**
 * @brief  在屏幕上画一列像素
 * @param  col    列坐标
 * @param  h      高度
 * @param  start  起始行
 * @param  color  颜色
 * @retval 无
 */
void isp_200_draw_col(uint8 col ,uint8 h,uint8 start,uint16 color)
{
    uint8 i=0;
    for(i=start;i<start+h;i++){
        ips200_draw_point(col,i,color);
    }
}

/**
 * @brief  在屏幕上画一行像素
 * @param  row    行坐标
 * @param  w      宽度
 * @param  start  起始列
 * @param  color  颜色
 * @retval 无
 */
void isp_200_draw_row(uint8 row ,uint8 w,uint8 start,uint16 color)
{
    uint8 i=0;
    for(i=start;i<start+w;i++){
        ips200_draw_point(i,row,color);
    }
}

/**
 * @brief  显示电机调试信息（编码器/PID参数），当前已注释
 * @param  无
 * @retval 无
 */
void display_motor(void)
{
//	ips200_show_string(0,0,"Encoder L R:");
//	ips200_show_int16(100,0,encoder_data_l);
//	ips200_show_int16(164,0,encoder_data_r);
//
//	ips200_show_string(0,32,"tar_speed:");
//	ips200_show_int16(80,32,tar_speed);
//
//	ips200_show_string(0,64,"L Kp Ki:");
//	ips200_show_int16(80,64,pi_l.kp_q);
//	ips200_show_int16(128,64,pi_l.ki_q);
//
//	ips200_show_string(0,96,"R Kp Ki:");
//	ips200_show_int16(80,96,pi_r.kp_q);
//	ips200_show_int16(128,96,pi_r.ki_q);
}
