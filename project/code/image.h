/**
 * @file    image.h
 * @brief   视觉图像处理头文件：边线搜索宏定义、全局变量声明、图像处理函数声明
 * @author  RRZ
 * @date    2026-05-16
 */

#ifndef __IMAGE_H_
#define __IMAGE_H_
#include "zf_common_headfile.h"

#define SEARCH_IMAGE_W      188     // 搜索图像宽度(列)
#define SEARCH_IMAGE_H      120     // 搜索图像高度(行)
#define BALACK_POINT        100     // 黑色参考点下限，低于此值视为黑线
#define WHITE_MAX_MUL        13     // 白点上限倍率(相对参考值/10)，越大白点判定越宽松
#define WHITE_MIN_MUL        7      // 白点下限倍率(相对参考值/10)，越小白点判定越严格
#define REFRENCE_ROW         4      // 参考曝光取底部多少行
#define SEARCH_RANGE         16     // 边线搜索左右窗口宽度
#define STOP_ROW             8      // 边线搜索停止行（顶部跳过行）
#define PIXEL_OFFSET         4      // 采样步长（行列间隔）

extern uint8 reference_point;
extern uint8 white_max_point;
extern uint8 white_min_point;
extern uint8 refenence_col_line[SEARCH_IMAGE_H];
extern uint8 reference_contrast_ratio;   // 边线和差比阈值(VOFA可调)，越大检测越严格
extern uint8 reference_col;
extern uint8 left_edge_line[SEARCH_IMAGE_H];
extern uint8 right_edge_line[SEARCH_IMAGE_H];
extern uint8 mid_line[SEARCH_IMAGE_H];
extern uint8 left_control_line[SEARCH_IMAGE_H];
extern uint8 right_control_line[SEARCH_IMAGE_H];
extern uint8 lost_left, lost_right;
extern uint8 cross_flag;
extern uint16 encoder_enter;
extern uint8 far image_copy[MT9V03X_H][MT9V03X_W];
extern uint8 binary_image[MT9V03X_H/2][MT9V03X_W/2];
extern uint8 th;
extern int32 err_sum;
extern int32 err_far, err_mid, err_near;
extern uint8 valid_start_row;
extern uint8 valid_row_count;
extern uint8 ff_cancelled;

// 圆环扫描方向: 0=无, 1=左环, 2=右环
extern uint8 ring_scan_dir;

// 斑马线检测
extern uint8 crosswalk_detected;
extern uint8 crosswalk_stop_flag;

// 十字路口检测
extern uint8 crossroad_detected;
extern uint8 cross_state;

// 原始边线采样（插值前保存，供拐点检测用）
extern uint8 left_edge_raw[SEARCH_IMAGE_H];
extern uint8 right_edge_raw[SEARCH_IMAGE_H];

// 前馈参数（VOFA可调）
extern float g_ff_scale;         // 前馈缩放系数
extern float g_ff_weight;        // 前馈融合权重，越大前馈占比越重
extern float g_diff_threshold;   // 远近误差差异阈值，用于冲突检测
extern float g_ff_quality_high;  // 前馈质量评估上限
extern float g_ff_quality_low;   // 前馈质量评估下限

extern int32 err_feedforward;
extern float ff_confidence;

// 圆环引导线（用于补线追踪）
extern uint8 ring_guide_line[SEARCH_IMAGE_H];
extern uint8 ring_guide_valid;

// ---- 核心视觉处理流程 ----
void get_reference_point(const uint8 *image);
void Search_reference_col(const uint8 *image);
void Search_line(const uint8 *image);
uint16 get_contrast(uint8 temp1, uint8 temp2);
void Fitted_Midline(void);
void insert_val(void);
void Error_sum(void);
void Extend_Lost_Line(void);
void Error_feedforward(void);
void Detect_Crosswalk(void);
void Detect_Crossroad(void);
void Scan_Ring_Path(uint8 is_left, uint8 start_row, uint8 start_col);
#endif
