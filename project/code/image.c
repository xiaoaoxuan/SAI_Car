/**
 * @file    image.c
 * @brief   视觉图像处理核心：参考点计算、边线搜索、中线合成、偏差计算、丢线外推、前馈、斑马线/十字路口检测
 * @author  RRZ
 * @date    2026-05-16
 */

#include "image.h"
#include "init.h"
#include "PID.h"

int32 err_feedforward = 0;
float ff_confidence = 1.0f;

float g_ff_scale = 1.5f;           // 前馈缩放系数(VOFA可调)
float g_ff_weight = 0.3f;          // 前馈基础权重(VOFA可调)
float g_diff_threshold = 35.0f;    // 前后段差值冲突阈值(VOFA可调)
float g_ff_quality_high = 0.8f;
float g_ff_quality_low = 0.5f;

extern uint8 ring_exit_compensate;
extern uint8 ring_compensate;
extern uint8 ring_comp_line[];
extern uint8 upper_turn_row;
extern uint8 exit_turn_row;
extern uint8 ring_state;
extern uint8 exit_turn_locked;

uint8 ff_cancelled = 0;

uint8 crosswalk_detected = 0;
uint8 crosswalk_stop_flag = 0;

uint8 crossroad_detected = 0;
uint8 cross_state = 0;
static uint8 cross_confirm_cnt = 0;

uint8 left_control_line[SEARCH_IMAGE_H];
uint8 right_control_line[SEARCH_IMAGE_H];

uint8 left_edge_raw[SEARCH_IMAGE_H];
uint8 right_edge_raw[SEARCH_IMAGE_H];

uint8 far image_copy[MT9V03X_H][MT9V03X_W];

/**
 * @brief  计算两像素的和差比对比度: |diff| * 200 / (sum + 1)，值越大差异越显著
 * @param  temp1  像素值1
 * @param  temp2  像素值2
 * @retval 对比度值
 * @note   和差比公式: contrast = |A-B| / (A+B) * 200
 *         分母+1防止除零；200是放大系数，使结果在0~200范围
 */
uint16 get_contrast(uint8 temp1, uint8 temp2)
{
    int16 diff = (int16)temp1 - (int16)temp2;                     // 带符号差值
    return  (uint16)((diff < 0 ? -diff : diff) * 200 / (temp1 + temp2 + 1));  // 和差比
}

uint8 reference_point;      // 参考曝光值（底部几行的平均灰度）
uint8 white_max_point;      // 白点上限（高于此值视为过曝/白色赛道）
uint8 white_min_point;      // 白点下限（低于此值视为黑色/暗区）

/**
 * @brief  从图像底部 REFRENCE_ROW 行计算参考曝光值，自适应光照
 * @param  image  原始灰度图像指针
 * @retval 无 (更新全局 reference_point / white_max_point / white_min_point)
 * @note   用底部几行的平均灰度作为曝光基准，
 *         white_max/white_min 通过倍率系数缩放并限幅到 [BALACK_POINT, 230]
 */
void get_reference_point(const uint8 *image)
{
    uint16 i;
    // 定位到底部 REFRENCE_ROW 行的起始位置
    uint8 *p = image + (SEARCH_IMAGE_H - REFRENCE_ROW) * SEARCH_IMAGE_W;
    uint16 temp = 0;
    uint32 tempSum = 0;

    // 统计底部 REFRENCE_ROW 行的像素总数
    temp = REFRENCE_ROW * SEARCH_IMAGE_W;

    // 累加所有像素值
    for (i = 0; i < temp; i++) {
        tempSum += *(p + i);
    }

    // 计算平均值作为参考曝光点
    reference_point = (uint8)(tempSum / temp);

    // 白点上限 = 参考值 * WHITE_MAX_MUL/10，下限 = 参考值 * WHITE_MIN_MUL/10
    // 限幅到 [BALACK_POINT, 230] 防止极端光照下溢出
    white_max_point = (uint8)func_limit_ab(reference_point * WHITE_MAX_MUL / 10, BALACK_POINT, 230);
    white_min_point = (uint8)func_limit_ab(reference_point * WHITE_MIN_MUL / 10, BALACK_POINT, 230);
}

uint8 refenence_col_line[SEARCH_IMAGE_H];
uint8 reference_contrast_ratio = 23;   // 边线和差比阈值，越大检测越严格
uint8 reference_col;

/**
 * @brief  搜索参考列：从底部向上扫，找和差比最早超阈值或到达 STOP_ROW 的列
 * @param  image  原始灰度图像指针
 * @retval 无 (更新 reference_col)
 * @note   逐列扫描，每列从底向上找最早出现"白点或高对比度"的位置，
 *         取位置最低的列作为参考列（即最靠近图像的赛道中心列）
 */
void Search_reference_col(const uint8 *image)
{
    uint8 col, row, temp1, temp2, i;
    uint8 globe_remote = 120, globe_remote_min = 120;   // 最远有效行和全局最小行
    uint16 contrast = 0;

    // 初始化为图像中心列
    reference_col = SEARCH_IMAGE_W / 2;

    // 圆环定向扫描模式下不更新参考列（使用环岛预设值）
    if (ring_scan_dir != 0) return;

    // ---- 逐列扫描 ----
    for (col = 0; col < SEARCH_IMAGE_W; col += PIXEL_OFFSET)
    {
        globe_remote = 120;

        // ---- 从底部向上扫描该列 ----
        for (row = SEARCH_IMAGE_H - 1; row > PIXEL_OFFSET; row -= PIXEL_OFFSET)
        {
            temp1 = *(image + row * SEARCH_IMAGE_W + col);                          // 当前像素
            temp2 = *(image + (row - PIXEL_OFFSET) * SEARCH_IMAGE_W + col);         // 上方像素

            if (temp2 > white_max_point)
            {
                // 上方像素太亮（过曝/白色赛道）→ 跳过此行
                continue;
            }
            else if (temp1 < white_min_point)
            {
                // 当前像素很暗（黑线）→ 记录位置并结束该列扫描
                if (globe_remote > row)
                    globe_remote = row;
                break;
            }

            // 计算和差比对比度
            contrast = get_contrast(temp1, temp2);

            // 对比度超阈值 或 到达顶部停止行 → 记录有效位置
            if (contrast > reference_contrast_ratio || row == STOP_ROW)
            {
                if (globe_remote > row)
                    globe_remote = row;
            }
        }

        // 取最靠底部（行号最大）的列作为参考列
        if (globe_remote < globe_remote_min)
        {
            globe_remote_min = globe_remote;
            reference_col = col;
        }
    }

    // 填充参考列数组（所有行统一使用该列）
    for (i = 0; i < SEARCH_IMAGE_H; i++)
        refenence_col_line[i] = reference_col;
}

uint8 left_edge_line[SEARCH_IMAGE_H];
uint8 right_edge_line[SEARCH_IMAGE_H];
uint8 mid_line[SEARCH_IMAGE_H];
uint8 lost_left, lost_right;     // 左右丢线行号

/**
 * @brief  双边搜索：和差比阈值检测赛道左右边线，两轮搜索+圆环定向扫描+出口补偿
 * @param  image  原始灰度图像指针
 * @retval 无 (更新 left_edge_line[] / right_edge_line[] / left_edge_raw[] / right_edge_raw[])
 * @note   核心搜索算法:
 *         1) 从底部向上逐行scan，对每行左右两侧分别用和差比搜索边线
 *         2) 两轮搜索：第一轮从参考列附近开始，第二轮从参考列重新搜索
 *         3) 找到边线后收缩搜索窗口(SEARCH_RANGE)，下一行在附近搜
 *         4) 圆环定向扫描：ring_scan_dir≠0时沿圆环方向找边线
 *         5) 出口补偿：ring_exit_compensate时保留出口拐点下的控制线
 */
void Search_line(const uint8 *image)
{
    const uint8 *p = image;
    uint8 row_max = SEARCH_IMAGE_H - 1;
    uint8 row_min = STOP_ROW;
    uint8 col_max = SEARCH_IMAGE_W - 1;
    uint8 col_min = 0;

    // 初始搜索起点：参考列左右各偏移5
    uint8 left_start_col  = reference_col - 5;
    uint8 right_start_col = reference_col + 5;
    uint8 left_end_col    = col_min;
    uint8 right_end_col   = SEARCH_IMAGE_W - 1;

    uint8 search_time;
    uint8 temp1, temp2;
    int16 contrast;

    uint8 left_stop = 0, right_stop = 0, stop_point = 0;
    uint8 col, row;

    // ---- 初始化：所有边线值设为默认（左=0贴边, 右=最右贴边）----
    lost_left  = STOP_ROW;
    lost_right = STOP_ROW;
    for (row = row_max; row >= row_min; row--)
    {
        left_edge_line[row]  = 0;
        right_edge_line[row] = SEARCH_IMAGE_W - 1;
    }

    // ---- 逐行扫描（从底部向上，步长 PIXEL_OFFSET）----
    for (row = row_max; row >= row_min; row -= PIXEL_OFFSET)
    {
        p = image + row * SEARCH_IMAGE_W;   // 当前行首地址

        // ========== 左侧边线搜索 ==========
        {
            search_time = 2;    // 两轮搜索
            do
            {
                // 第二轮：从参考列重新开始搜（扩大搜索范围）
                if (search_time == 1)
                {
                    left_start_col = reference_col;
                    left_end_col   = col_min;
                }
                search_time--;

                // 从 start_col 向左侧扫描
                for (col = left_start_col; col > left_end_col + PIXEL_OFFSET; col -= PIXEL_OFFSET)
                {
                    temp1 = *(p + col);                              // 当前像素
                    temp2 = *(p + col - PIXEL_OFFSET);              // 左侧像素

                    // 起始列就是黑点 → 丢线标记
                    if (temp1 < white_min_point && col == left_start_col && left_start_col == reference_col)
                    {
                        // 该列起始就是黑色 → 整段丢线
                        for (stop_point = row; stop_point > 1; stop_point--)
                            left_edge_line[stop_point] = col_min;
                        lost_left = row;
                        search_time = 0;
                        break;
                    }

                    // 当前像素是黑点 → 找到边线
                    if (temp1 < white_min_point)
                    {
                        left_edge_line[row] = col;
                        break;
                    }

                    // 左侧像素是白点 → 跳过（不在边线上）
                    if (temp2 > white_max_point) continue;

                    // 和差比检测：当前暗、左侧亮 = 赛道边界
                    contrast = get_contrast(temp1, temp2);
                    if (contrast > reference_contrast_ratio || col == col_min)
                    {
                        left_edge_line[row] = col;

                        // 收缩搜索窗口到该列附近（±SEARCH_RANGE）
                        left_start_col = (uint8)func_limit_ab(col + SEARCH_RANGE, col, col_max);
                        left_end_col   = (uint8)func_limit_ab(col - SEARCH_RANGE, col_min, col);
                        search_time = 0;
                        break;
                    }
                }
            } while (search_time);
        }

        // ========== 右侧边线搜索 ==========
        {
            search_time = 2;
            do
            {
                if (search_time == 1)
                {
                    right_start_col = reference_col;
                    right_end_col   = col_max;
                }
                search_time--;

                // 从 start_col 向右侧扫描
                for (col = right_start_col; col <= right_end_col - PIXEL_OFFSET; col += PIXEL_OFFSET)
                {
                    temp1 = *(p + col);                              // 当前像素
                    temp2 = *(p + col + PIXEL_OFFSET);              // 右侧像素

                    // 起始列就是黑点 → 丢线标记
                    if (temp1 < white_min_point && col == right_start_col && right_start_col == reference_col)
                    {
                        for (stop_point = row; stop_point > row_min; stop_point--)
                            right_edge_line[stop_point] = SEARCH_IMAGE_W - 1;
                        lost_right = row;
                        search_time = 0;
                        break;
                    }

                    // 当前像素是黑点 → 找到边线
                    if (temp1 < white_min_point)
                    {
                        right_edge_line[row] = col;
                        break;
                    }

                    if (temp2 > white_max_point) continue;

                    // 和差比检测
                    contrast = get_contrast(temp1, temp2);
                    if (contrast > reference_contrast_ratio || col >= col_max - PIXEL_OFFSET)
                    {
                        right_edge_line[row] = col;

                        // 收缩搜索窗口
                        right_start_col = (uint8)func_limit_ab(col - SEARCH_RANGE, col_min, col);
                        right_end_col   = (uint8)func_limit_ab(col + SEARCH_RANGE, col, col_max);
                        search_time = 0;
                        break;
                    }
                }
            } while (search_time);
        }
    }

    // ---- 保存原始采样边线（插值前，供拐点检测用）----
    memcpy(left_edge_raw,  left_edge_line,  sizeof(left_edge_line));
    memcpy(right_edge_raw, right_edge_line, sizeof(right_edge_line));

    // 填充采样间隔之间的行（保持原始值的连续性）
    {
        uint8 r, j;
        for (r = SEARCH_IMAGE_H - 1; r >= STOP_ROW + PIXEL_OFFSET; r -= PIXEL_OFFSET) {
            for (j = 1; j < PIXEL_OFFSET; j++) {
                left_edge_raw[r - j] = left_edge_raw[r];
                right_edge_raw[r - j] = right_edge_raw[r];
            }
        }
    }

    // ---- 圆环定向扫描：沿环岛方向重新搜索边线 ----
    if (ring_scan_dir == 1)
    {
        // 左环扫描：从 upper_base_col 向右找亮→暗的跳变（赛道左边界）
        for (row = row_max; row >= row_min; row--)
        {
            p = image + row * SEARCH_IMAGE_W;
            for (col = upper_base_col; col < col_max; col++)
            {
                // 左边亮(赛道) + 右边暗(背景) = 边界
                if (*(p + col) > white_min_point && *(p + col + 1) <= white_min_point)
                {
                    left_edge_line[row] = col;
                    break;
                }
            }
            if (col >= col_max)   // 未找到 → 默认贴边
                left_edge_line[row] = 0;
        }
    }
    else if (ring_scan_dir == 2)
    {
        // 右环扫描：从 upper_base_col 向左找亮→暗的跳变（赛道右边界）
        for (row = row_max; row >= row_min; row--)
        {
            p = image + row * SEARCH_IMAGE_W;
            for (col = upper_base_col; col > col_min; col--)
            {
                // 右边亮(赛道) + 左边暗(背景) = 边界
                if (*(p + col) > white_min_point && *(p + col - 1) <= white_min_point)
                {
                    right_edge_line[row] = col;
                    break;
                }
            }
            if (col == col_min)
                right_edge_line[row] = SEARCH_IMAGE_W - 1;
        }
    }

    // ---- 出口补偿：出口拐点以下保留原始控制线（不覆盖为补偿线）----
    if (ring_exit_compensate) {
        if (ring_scan_dir == 1) {
            for (row = row_max; row >= row_min; row--) {
                if (row > exit_turn_row)           // 出口拐点以下
                    left_edge_line[row] = left_control_line[row];   // 保留原始边线
            }
        }
        else if (ring_scan_dir == 2) {
            for (row = row_max; row >= row_min; row--) {
                if (row > exit_turn_row)
                    right_edge_line[row] = right_control_line[row];
            }
        }
    }

    // ---- 后处理：插值填充 + 更新控制线 ----
    insert_val();   // 线性插值填充采样间隔
    memcpy(left_control_line,  left_edge_line,  sizeof(left_edge_line));
    memcpy(right_control_line, right_edge_line, sizeof(right_edge_line));
}

/**
 * @brief  对边线采样行之间线性插值填充，使 PIXEL_OFFSET 间隔变为连续线
 * @param  无
 * @retval 无 (原地修改 left_edge_line[] / right_edge_line[])
 * @note   从底部向上，对每两个采样行之间按等步长线性插值，
 *         使得边线数组的每一行都有连续的列坐标
 */
void insert_val(void)
{
    uint8 i, j;
    uint8 front_p, cur_p;
    uint8 step;

    // ---- 左侧边线插值 ----
    front_p = left_edge_line[SEARCH_IMAGE_H - 1];    // 最后一行(底部)作为起点
    for (i = SEARCH_IMAGE_H - PIXEL_OFFSET - 1; i >= STOP_ROW; i -= PIXEL_OFFSET) {
        cur_p = left_edge_line[i];                    // 上一采样行的列值

        if (cur_p >= front_p) {
            // 方向：向外(列增大) → 等步长递增
            step = (cur_p - front_p + PIXEL_OFFSET - 1) / PIXEL_OFFSET;
            for (j = 1; j < PIXEL_OFFSET; j++) {
                left_edge_line[i - j + PIXEL_OFFSET] = front_p + step * j;
            }
        }
        else {
            // 方向：向内(列减小) → 等步长递减
            step = (front_p - cur_p + PIXEL_OFFSET - 1) / PIXEL_OFFSET;
            for (j = 1; j < PIXEL_OFFSET; j++) {
                left_edge_line[i - j + PIXEL_OFFSET] = front_p - step * j;
            }
        }
        front_p = cur_p;   // 更新起点为当前采样行
    }

    // ---- 右侧边线插值（逻辑同左侧）----
    front_p = right_edge_line[SEARCH_IMAGE_H - 1];
    for (i = SEARCH_IMAGE_H - PIXEL_OFFSET - 1; i >= STOP_ROW; i -= PIXEL_OFFSET) {
        cur_p = right_edge_line[i];

        if (cur_p >= front_p) {
            step = (cur_p - front_p + PIXEL_OFFSET - 1) / PIXEL_OFFSET;
            for (j = 1; j < PIXEL_OFFSET; j++) {
                right_edge_line[i - j + PIXEL_OFFSET] = front_p + j * step;
            }
        }
        else {
            step = (front_p - cur_p + PIXEL_OFFSET - 1) / PIXEL_OFFSET;
            for (j = 1; j < PIXEL_OFFSET; j++) {
                right_edge_line[i - j + PIXEL_OFFSET] = front_p - j * step;
            }
        }
        front_p = cur_p;
    }
}

/**
 * @brief  由左右边线合成中线，支持圆环补线和出口补偿
 * @param  无
 * @retval 无 (更新 mid_line[])
 * @note   正常模式: mid_line = (left + right) / 2
 *         入环补线: 拐点以上用原始边线，拐点以下用补偿线替代丢线侧
 *         出口补偿: 拐点以上用补偿线，拐点以下用原始控制线
 */
void Fitted_Midline(void)
{
    uint8 i;
    uint8 comp_val;

    if (ring_exit_compensate) {
        // 出口补偿模式：拐点以上用补偿线，拐点以下用原始线
        if (ring_scan_dir == 1) {
            for (i = 0; i < SEARCH_IMAGE_H; i++) {
                comp_val = (i <= exit_turn_row) ? ring_comp_line[i] : left_control_line[i];
                mid_line[i] = (comp_val + right_control_line[i]) >> 1;   // 均值=中线
            }
        } else {
            for (i = 0; i < SEARCH_IMAGE_H; i++) {
                comp_val = (i <= exit_turn_row) ? ring_comp_line[i] : right_control_line[i];
                mid_line[i] = (left_control_line[i] + comp_val) >> 1;
            }
        }
    } else if (ring_compensate) {
        // 入环补线模式：拐点以下用补偿线替代丢线侧
        if (ring_scan_dir == 1) {
            for (i = 0; i < SEARCH_IMAGE_H; i++) {
                comp_val = (i >= upper_turn_row) ? ring_comp_line[i] : left_control_line[i];
                mid_line[i] = (comp_val + right_control_line[i]) >> 1;
            }
        } else {
            for (i = 0; i < SEARCH_IMAGE_H; i++) {
                comp_val = (i >= upper_turn_row) ? ring_comp_line[i] : right_control_line[i];
                mid_line[i] = (left_control_line[i] + comp_val) >> 1;
            }
        }
    } else {
        // 正常模式：直接取双边均值
        for (i = 0; i < SEARCH_IMAGE_H; i++)
            mid_line[i] = (left_control_line[i] + right_control_line[i]) >> 1;
    }
}

int32 err_sum = 0;             // 加权中线偏差和（供PID使用）
int32 err_far = 0, err_mid = 0, err_near = 0;   // 远/中/近三分区误差
uint8 valid_start_row = 80;    // 有效反馈起始行
uint8 valid_row_count = 26;    // 有效反馈行数

static uint8 is_line_invalid(uint8 row);

/**
 * @brief  三分区加权偏差：远/中/近加权求和，自动跳过无效行
 * @param  无
 * @retval 无 (更新 err_sum / valid_start_row / valid_row_count)
 * @note   正常模式: 30/30/40 权重, 环岛模式: 25/30/45 权重
 *         先扫描中线跳变确定有效起始行，再分三个区域加权平均
 */
void Error_sum(void)
{
    uint8 i;
    int32 err1 = 0, err2 = 0, err3 = 0;
    int32 cnt1 = 0, cnt2 = 0, cnt3 = 0;
    uint8 first_valid, block_rows, p1_end, p2_end;

    // ========== 出口模式：从出口拐点往上取有效区域 ==========
    if (ring_state == 4 && exit_turn_locked) {
        uint8 zone_end = exit_turn_row + 8;
        if (zone_end > 106) zone_end = 106;

        first_valid = 0;
        valid_start_row = first_valid;
        block_rows = zone_end - first_valid;

        // 40%/30%/30% 分区
        p1_end = first_valid + block_rows * 40 / 100;
        p2_end = first_valid + block_rows * 70 / 100;

        // 近区（底部40%）
        for (i = first_valid; i < p1_end; i++) { err1 += mid_line[i]; cnt1++; }
        // 中区（中间30%）
        for (i = p1_end; i < p2_end; i++) { err2 += mid_line[i]; cnt2++; }
        // 远区（顶部30%）
        for (i = p2_end; i < zone_end; i++) { err3 += mid_line[i]; cnt3++; }

        // 各区域取均值
        if (cnt1 > 0) err1 /= cnt1;
        if (cnt2 > 0) err2 /= cnt2;
        if (cnt3 > 0) err3 /= cnt3;

        // 加权融合：近区重视(45%) + 中区30% + 远区25%
        err_sum = (err1 * 25 + err2 * 30 + err3 * 45) / 100;
        valid_row_count = (uint8)(cnt1 + cnt2 + cnt3);
        return;
    }

    // ========== 入环补线模式：从上拐点开始取有效区域 ==========
    if (ring_state == 2) {
        first_valid = upper_turn_row;
        if (first_valid < 6) first_valid = 6;
        valid_start_row = first_valid;
        block_rows = 106 - first_valid;

        p1_end = first_valid + block_rows * 40 / 100;
        p2_end = first_valid + block_rows * 70 / 100;

        // 近中区跳过无效行（双边丢线+中心暗），远区全部计入
        for (i = first_valid; i < p1_end; i++) {
            if (!is_line_invalid(i)) { err1 += mid_line[i]; cnt1++; }
        }
        for (i = p1_end; i < p2_end; i++) {
            if (!is_line_invalid(i)) { err2 += mid_line[i]; cnt2++; }
        }
        for (i = p2_end; i < 106; i++) { err3 += mid_line[i]; cnt3++; }

        if (cnt1 > 0) err1 /= cnt1;
        if (cnt2 > 0) err2 /= cnt2;
        if (cnt3 > 0) err3 /= cnt3;

        err_sum = (err1 * 25 + err2 * 30 + err3 * 45) / 100;
        valid_row_count = (uint8)(cnt1 + cnt2 + cnt3);
        return;
    }

    // ========== 正常模式：扫描中线跳变确定有效起始行 ==========
    first_valid = 0;
    i = 79;
    do {
        // 无效行条件：双边丢线+中心暗 或 中线跳变>40
        if (is_line_invalid(i)) {
            first_valid = i + 1;   // 从下一行开始有效
            break;
        }
        if (i + 4 < 106) {
            int16 jump = (int16)mid_line[i] - (int16)mid_line[i + 4];
            if (jump < -40 || jump > 40) {   // 4行内跳变>40 → 此行为噪点
                first_valid = i + 1;
                break;
            }
        }
    } while (i-- > 0);

    valid_start_row = first_valid;
    block_rows = 106 - first_valid;

    // 40%/30%/30% 分区
    p1_end = first_valid + block_rows * 40 / 100;
    p2_end = first_valid + block_rows * 70 / 100;

    // 近区（底部40%）
    for (i = first_valid; i < p1_end; i++) {
        if (!is_line_invalid(i)) { err1 += mid_line[i]; cnt1++; }
    }
    // 中区（中间30%）
    for (i = p1_end; i < p2_end; i++) {
        if (!is_line_invalid(i)) { err2 += mid_line[i]; cnt2++; }
    }
    // 远区（顶部30%，全部计入）
    for (i = p2_end; i < 106; i++) { err3 += mid_line[i]; cnt3++; }

    // 各区域取均值
    if (cnt1 > 0) err1 /= cnt1;
    if (cnt2 > 0) err2 /= cnt2;
    if (cnt3 > 0) err3 /= cnt3;

    // 加权融合：近区30% + 中区30% + 远区40%（正常模式远区权重较高）
    err_sum = (err1 * 30 + err2 * 30 + err3 * 40) / 100;

    valid_row_count = (uint8)(cnt1 + cnt2 + cnt3);
}

/**
 * @brief  丢线外推：根据最后有效行斜率预测丢线区域边线位置
 * @param  无
 * @retval 无 (修改 left_control_line[] / right_control_line[])
 * @note   从丢线行开始向上，用底部的边线斜率线性外推，保持边线方向连续性
 */
void Extend_Lost_Line(void)
{
    uint8 row;
    int16 predicted;
    float slope;

    // ---- 左侧丢线外推 ----
    if (lost_left != STOP_ROW)
    {
        // 找到丢线行下方第一个有效行作为基准行
        uint8 base_row = lost_left;
        while (base_row < SEARCH_IMAGE_H - 1)
        {
            base_row++;
            if (left_edge_line[base_row] > 0 && left_edge_line[base_row] < SEARCH_IMAGE_W - 1)
                break;
        }

        if (base_row < SEARCH_IMAGE_H - 1)
        {
            // 计算斜率 = (基准行 - 上一采样行) / PIXEL_OFFSET
            int16 d_col = (int16)left_edge_line[base_row] - (int16)left_edge_line[base_row - PIXEL_OFFSET];
            slope = (float)d_col / PIXEL_OFFSET;

            // 从丢线行向上按斜率外推
            for (row = lost_left; row >= STOP_ROW; row--)
            {
                predicted = (int16)left_edge_line[base_row] + (int16)(slope * (row - base_row));
                if (predicted < 0) predicted = 0;
                if (predicted > SEARCH_IMAGE_W - 1) predicted = SEARCH_IMAGE_W - 1;
                left_control_line[row] = (uint8)predicted;
            }
        }
    }

    // ---- 右侧丢线外推（逻辑同左侧）----
    if (lost_right != STOP_ROW)
    {
        uint8 base_row = lost_right;
        while (base_row < SEARCH_IMAGE_H - 1)
        {
            base_row++;
            if (right_edge_line[base_row] > 0 && right_edge_line[base_row] < SEARCH_IMAGE_W - 1)
                break;
        }

        if (base_row < SEARCH_IMAGE_H - 1)
        {
            int16 d_col = (int16)right_edge_line[base_row] - (int16)right_edge_line[base_row - PIXEL_OFFSET];
            slope = (float)d_col / PIXEL_OFFSET;

            for (row = lost_right; row >= STOP_ROW; row--)
            {
                predicted = (int16)right_edge_line[base_row] + (int16)(slope * (row - base_row));
                if (predicted < 0) predicted = 0;
                if (predicted > SEARCH_IMAGE_W - 1) predicted = SEARCH_IMAGE_W - 1;
                right_control_line[row] = (uint8)predicted;
            }
        }
    }
}

/**
 * @brief  判断某行中线是否无效（双边丢线且中心点偏黑<50）
 * @param  row  行号
 * @retval 1=无效, 0=有效
 */
static uint8 is_line_invalid(uint8 row)
{
    // 双边都贴边（完全丢线）
    if (left_control_line[row] <= 5 && right_control_line[row] >= SEARCH_IMAGE_W - 5)
    {
        // 且中心点灰度<50（极暗，说明不是赛道）
        if (mt9v03x_image[row][mid_line[row]] < 50)
            return 1;
    }
    return 0;
}

/**
 * @brief  前馈误差重置（每帧开始时清零，由 Error_sum 后的 Error_feedforward 重新计算）
 * @param  无
 * @retval 无
 */
void Error_feedforward(void)
{
    ff_confidence = 0.0f;
    err_feedforward = 0;
    ff_cancelled = 0;
}

/**
 * @brief  斑马线检测：50-57行区域扫描短黑段(2~7像素)≥4个即判定
 * @param  无
 * @retval 无 (设置 crosswalk_detected / crosswalk_stop_flag)
 * @note   斑马线特征：赛道内出现多个短而离散的黑色条纹(2-7像素宽)
 */
void Detect_Crosswalk(void)
{
    uint8 row, col;
    uint8 black_len, short_black_cnt;
    uint8 l_edge, r_edge, width;

    crosswalk_detected = 0;

    // 扫描50-57行（斑马线通常在这个高度区域）
    for (row = 50; row <= 57; row++)
    {
        l_edge = left_control_line[row];
        r_edge = right_control_line[row];
        if (l_edge >= r_edge) continue;           // 无效行

        width = r_edge - l_edge;
        if (width < 25) continue;                  // 赛道太窄，不可靠

        black_len = 0;
        short_black_cnt = 0;

        // 从左到右扫描该行赛道内的黑色段
        for (col = l_edge; col <= r_edge; col++)
        {
            if (mt9v03x_image[row][col] < white_min_point)
            {
                black_len++;                       // 持续暗像素
            }
            else
            {
                // 遇到亮像素：检查上一个暗段的长度
                if (black_len >= 2 && black_len <= 7)
                {
                    short_black_cnt++;              // 短黑段计数+1
                }
                black_len = 0;                      // 重置段长度
            }
        }

        // 一行内≥4个短黑段 → 判定为斑马线
        if (short_black_cnt >= 4)
        {
            crosswalk_detected = 1;
            crosswalk_stop_flag = 1;               // 触发停车标志
            return;
        }
    }
}

/**
 * @brief  十字路口检测：30-90行双边丢线≥15行 + 近场有单边线，连续2帧确认
 * @param  无
 * @retval 无 (设置 crossroad_detected / cross_state)
 * @note   十字特征：中上部大范围双边丢线（十字交叉区），但底部仍有单侧边线
 */
void Detect_Crossroad(void)
{
    uint8 row;
    uint8 bilateral_lost = 0;     // 双边丢线计数
    uint8 near_ok = 0;            // 近场有效标志

    crossroad_detected = 0;

    // ---- 1. 检查近场（100-119行）是否有单边线 ----
    for (row = 100; row <= 119; row++)
    {
        if (left_control_line[row] > 10 || right_control_line[row] < SEARCH_IMAGE_W - 10)
        {
            near_ok = 1;   // 至少有一侧边线在赛道内
            break;
        }
    }
    if (!near_ok) { cross_confirm_cnt = 0; return; }   // 近场全丢 → 非十字

    // ---- 2. 统计30-90行双边丢线行数 ----
    for (row = 30; row <= 90; row++)
    {
        if (left_control_line[row] <= 5 && right_control_line[row] >= SEARCH_IMAGE_W - 5)
        {
            bilateral_lost++;
        }
    }

    // ---- 3. 双边丢线≥15行 → 十字路口候选 ----
    if (bilateral_lost >= 15)
    {
        cross_confirm_cnt++;
        if (cross_confirm_cnt >= 2)          // 连续2帧确认（防抖动）
        {
            crossroad_detected = 1;
            cross_state = 1;
            cross_confirm_cnt = 0;
        }
    }
    else
    {
        if (cross_confirm_cnt > 0) cross_confirm_cnt--;   // 不满足条件则衰减计数
    }
}
