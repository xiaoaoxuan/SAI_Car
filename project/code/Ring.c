/**
 * @file    Ring.c
 * @brief   圆环/环岛检测5状态机：空闲→下拐确认→入环补线→等待出口→寻找出口→复位
 * @author  RRZ
 * @date    2026-05-16
 */

#include "Ring.h"
#include "image.h"
#include "zf_common_headfile.h"

uint8 current_step = 0;
uint8 ring_state = 0;
uint8 ring_scan_dir = 0;
uint8 ring_l = 0, ring_r = 0;
uint8 ring_compensate = 0;
uint8 upper_turn_row = 0;
uint8 upper_base_col = 0;
uint8 low_turn_confirmed = 0;
int32 gyro_accum = 0;
uint8 debug_comp_row = 0;
uint8 debug_comp_start_col = 0;
uint8 debug_ring_dir = 0;
uint8 old_reference_col = 0;
uint8 ring_disable_feedforward = 0;
uint8 ring_exit_compensate = 0;
uint16 ring_cooldown = 0;
uint8 ring_comp_line[SEARCH_IMAGE_H];

int16 ring_force_steer = 0;

uint8 exit_turn_row = 0, exit_turn_col = 0;
uint8 exit_turn_locked = 0;

uint8 low_turn_row = 0, low_turn_col = 0;
static uint8 high_turn_row = 0, high_turn_col = 0;
static uint8 gyro_integrating = 0;
static int32 exit_gyro_threshold = 0;
static uint16 state1_timeout = 0;

static uint8 exit_confirm_cnt = 0;
static uint8 state2_no_turn_cnt = 0;

extern float diff_kp;
extern float g_ring_diff_kp;
static float saved_diff_kp;
static uint8 diff_kp_modified = 0;

static uint8 cr_confirm = 0;

static uint8 Check_High_Turn(uint8 is_left, uint8 *row, uint8 *col);
static uint8 Check_Low_Turn(uint8 is_left, uint8 *row, uint8 *col, uint8 outward_only);
static uint8 Check_Cross_Low_Turn(uint8 is_left, uint8 *row, uint8 *col);

/**
 * @brief  十字路口屏蔽：双侧下拐点行号差<30视为十字，连续2帧确认后复位环状态机
 */
static void Check_Crossroad_Guard(void)
{
    uint8 r1, c1, r2, c2;
    uint8 detected = 0;

    // 双侧同时检测下拐点
    if (Check_Cross_Low_Turn(0, &r1, &c1) && Check_Cross_Low_Turn(1, &r2, &c2)) {
        // 双侧下拐行号差<30 → 判定为十字（非环岛）
        if (abs((int16)r1 - (int16)r2) < 30) {
            detected = 1;
        }
    }

    if (detected) {
        cr_confirm++;
        if (cr_confirm >= 2) {   // 连续2帧确认
            Ring_Init();          // 复位环状态机，避免误入环
            cr_confirm = 0;
        }
    } else {
        cr_confirm = 0;           // 不连续则清零
    }
}

/**
 * @brief  统计15~119行边线丢线率
 * @param  is_left  1=左线, 0=右线
 * @retval 丢线百分比(0-100)
 */
static uint8 Edge_Lost_Ratio(uint8 is_left) {
    uint8 *edge = is_left ? left_control_line : right_control_line;
    uint8 threshold = is_left ? 5 : (SEARCH_IMAGE_W - 5);   // 贴边阈值
    uint16 lost = 0, total = 0;
    uint8 i;

    // 遍历15~119行
    for (i = 15; i <= 119; i++) {
        total++;
        if (is_left) {
            if (edge[i] <= threshold) lost++;    // 左线贴左边界=丢线
        } else {
            if (edge[i] >= threshold) lost++;    // 右线贴右边界=丢线
        }
    }

    if (total == 0) return 0;
    return (uint8)(lost * 100 / total);          // 返回百分比
}

/**
 * @brief  画入环补偿线：从上拐点斜向延伸到对侧边界
 */
static void Draw_Compensate_Line(uint8 row, uint8 col, uint8 is_left) {
    uint8 target_col = is_left ? 0 : (SEARCH_IMAGE_W - 1);    // 补偿目标：对侧边界
    uint8 start_col;
    uint8 i;
    int16 delta_row, delta_col;
    int32 new_col;
    int16 rows_from_start;

    // 补偿起始列：从本侧远端开始
    start_col = is_left ? (uint8)(SEARCH_IMAGE_W)
                        : (uint8)(SEARCH_IMAGE_W * 0);

    debug_comp_start_col = start_col;

    // 计算斜率
    delta_row = 119 - row;                     // 从上拐点到画面底部
    if (delta_row <= 0) return;
    delta_col = (int16)target_col - (int16)start_col;

    // 从上拐点行到119行：线性插值画斜线
    for (i = row; i <= 119; i++) {
        rows_from_start = (int16)(i - row);
        new_col = (int32)start_col + ((int32)delta_col * rows_from_start) / delta_row;
        if (is_left) {
            if (new_col < 0) new_col = 0;
        } else {
            if (new_col > SEARCH_IMAGE_W - 1) new_col = SEARCH_IMAGE_W - 1;
        }
        ring_comp_line[i] = (uint8)new_col;
    }

    // 上拐点之前：填充为本侧边界
    for (i = 0; i < row; i++) {
        ring_comp_line[i] = (is_left ? 0 : SEARCH_IMAGE_W - 1);
    }
}

/**
 * @brief  画出环补偿线：从出口拐点斜向延伸到远侧边界
 */
static void Draw_Exit_Compensate_Line(uint8 row, uint8 col, uint8 is_left) {
    uint8 target_col = is_left ? (SEARCH_IMAGE_W - 1) : 0;    // 远侧边界
    uint8 i;
    int16 delta_row, delta_col;
    int32 new_col;
    int16 rows_from_start;

    // 从出口拐点向上延伸到0行
    delta_row = row;
    if (delta_row <= 0) return;
    delta_col = (int16)target_col - (int16)col;

    // 从出口拐点行到0行：线性插值画斜线
    for (i = 0; i <= row; i++) {
        rows_from_start = (int16)(row - i);      // 距离出口拐点的行数
        new_col = (int32)col + ((int32)delta_col * rows_from_start) / delta_row;
        if (is_left) {
            if (new_col < 0) new_col = 0;
        } else {
            if (new_col > SEARCH_IMAGE_W - 1) new_col = SEARCH_IMAGE_W - 1;
        }
        ring_comp_line[i] = (uint8)new_col;
    }

    // 出口拐点之后：边界保持
    for (i = row + 1; i <= 119; i++) {
        ring_comp_line[i] = (is_left ? 0 : SEARCH_IMAGE_W - 1);
    }
}

/**
 * @brief  圆环状态机复位：清零所有状态变量，恢复差速系数
 */
void Ring_Init(void) {
    // 清零状态变量
    ring_state = 0;
    ring_scan_dir = 0;
    ring_l = 0; ring_r = 0;
    ring_compensate = 0;
    ring_exit_compensate = 0;
    upper_turn_row = 0;
    upper_base_col = 0;
    low_turn_confirmed = 0;
    gyro_accum = 0;
    gyro_integrating = 0;
    debug_ring_dir = 0;
    debug_comp_row = 0;
    old_reference_col = 0;
    ring_disable_feedforward = 0;
    exit_turn_locked = 0;
    exit_turn_row = 0;
    exit_turn_col = 0;
    exit_gyro_threshold = 0;
    state1_timeout = 0;
    ring_cooldown = 0;
    ring_force_steer = 0;
    exit_confirm_cnt = 0;
    state2_no_turn_cnt = 0;

    // 清零补偿线数组
    memset(ring_comp_line, 0, sizeof(ring_comp_line));

    // 恢复原始差速系数（入环时被替换为g_ring_diff_kp）
    if (diff_kp_modified) {
        diff_kp = saved_diff_kp;
        diff_kp_modified = 0;
    }
}

/**
 * @brief  陀螺仪积分（每5ms调用）：仅在 gyro_integrating=1 时累加
 * @note   用于测量入环后的转向量，判断是否转够了角度
 */
void Gyro_Integrate(void) {
    if (gyro_integrating) {
        // 将陀螺仪原始值转为度/秒（÷16.4是IMU660RA的比例因子）
        float gyro_dps = (float)imu660ra_gyro_z / 16.4f;
        // 累加角速度绝对值 * 5ms = 角度增量
        gyro_accum += (int32)(fabs(gyro_dps) * 0.5f);
    }
}

/**
 * @brief  判断边线连续（相邻采样点差值≤40）
 */
uint8 isContinueLine(uint8* arr, uint8 start_row, uint8 end_row) {
    int16 i;
    // 从start_row向下扫描到end_row，检查相邻采样点的列偏差
    for (i = start_row; i > end_row; i -= PIXEL_OFFSET)
        if (abs(arr[i] - arr[i - PIXEL_OFFSET]) > 40)   // 跳变>40视为不连续
            return 0;
    return 1;
}

/**
 * @brief  判断边线为直线（不贴边 + 总偏移≤行差 + 各段斜率差≤6 + 有效点≥一半）
 */
uint8 IsLineStraight(uint8 *arr, uint8 start, uint8 end) {
    int16 i;
    int16 dy;
    int16 dx_total;
    int16 first_diff = 0;
    uint8 valid_count = 0;
    uint8 total_count = 0;
    uint8 straight = 1;

    // 起点或终点贴边 → 非直线
    if (arr[start] <= 5 || arr[start] >= SEARCH_IMAGE_W - 5 ||
        arr[end] <= 5 || arr[end] >= SEARCH_IMAGE_W - 5) {
        return 0;
    }

    // 总偏移量检查：左右偏移不超过行数差
    dy = start - end;
    dx_total = (int16)arr[start] - (int16)arr[end];
    if (abs(dx_total) > dy) {
        return 0;
    }

    // 逐段检查斜率一致性
    for (i = start; i > end + PIXEL_OFFSET; i -= PIXEL_OFFSET) {
        total_count++;
        if (arr[i] <= 5 || arr[i] >= SEARCH_IMAGE_W - 5) continue;   // 跳过贴边点
        valid_count++;
        if (valid_count == 1) {
            first_diff = (int16)arr[i - PIXEL_OFFSET] - (int16)arr[i];   // 记录首段斜率
        } else {
            int16 cur_diff = (int16)arr[i - PIXEL_OFFSET] - (int16)arr[i];
            if (abs(cur_diff - first_diff) > 6) {   // 斜率差>6 → 非直线
                straight = 0;
                break;
            }
        }
    }

    // 有效采样点不足一半 → 数据不可靠
    if (valid_count < total_count / 2) return 0;
    return straight;
}

/**
 * @brief  底部(105~119行)丢线统计
 */
uint16 Count_Lost_Lines(uint8 is_left) {
    uint16 i, cnt = 0;
    uint8 *edge = is_left ? left_edge_line : right_edge_line;
    uint8 threshold = is_left ? 5 : (SEARCH_IMAGE_W - 5);
    for (i = 105; i <= 119; i++) {
        if (is_left) {
            if (edge[i] < threshold) cnt++;
        } else {
            if (edge[i] > threshold) cnt++;
        }
    }
    return cnt;
}

/**
 * @brief  全图丢线统计
 */
static uint16 Count_Lost_Lines_Full(uint8 is_left) {
    uint16 i, cnt = 0;
    uint8 *edge = is_left ? left_edge_line : right_edge_line;
    uint8 threshold = is_left ? 5 : (SEARCH_IMAGE_W - 5);
    for (i = 0; i < SEARCH_IMAGE_H; i++) {
        if (is_left) {
            if (edge[i] < threshold) cnt++;
        } else {
            if (edge[i] > threshold) cnt++;
        }
    }
    return cnt;
}

/**
 * @brief  下拐点检测：边线由内向外跳变（从赛道中心向外侧突变）
 * @note   搜索119→30+2*PIXEL_OFFSET，间隔2*PIXEL_OFFSET行差值>6；下方6行须为直线
 *         左环: top - under < -6（向左跳变）; 右环: top - under > 6（向右跳变）
 */
static uint8 Check_Low_Turn(uint8 is_left, uint8 *row, uint8 *col, uint8 outward_only) {
    uint8 i, top, under;
    uint8 *edge = is_left ? left_edge_raw : right_edge_raw;
    int16 diff;

    // 从底部(119行)向上扫描到30+2*PIXEL_OFFSET
    for (i = 119; i > 30 + 2*PIXEL_OFFSET; i -= PIXEL_OFFSET) {
        top   = edge[i - 2*PIXEL_OFFSET];   // 上采样点
        under = edge[i];                      // 下采样点
        diff = (int16)top - (int16)under;

        if (outward_only) {
            // 仅检测向外跳变：左线向左(差值负数)，右线向右(差值正数)
            if (is_left) {
                if (diff >= -6) continue;     // 左线要求 top - under < -6
            } else {
                if (diff <= 6) continue;      // 右线要求 top - under > 6
            }
        } else {
            // 双向检测：绝对值>6即可
            if (diff < 0) diff = -diff;
            if (diff <= 6) continue;
        }

        // 下方6行须为直线（排除噪声）
        if (!IsLineStraight(edge, i + 6, i)) continue;

        *row = i;       // 返回拐点行
        *col = under;   // 返回拐点列
        return 1;
    }
    return 0;
}

/**
 * @brief  十字专用下拐点：搜索20~70行，仅从内向外>10
 */
static uint8 Check_Cross_Low_Turn(uint8 is_left, uint8 *row, uint8 *col) {
    uint8 i, top, under;
    uint8 *edge = is_left ? left_edge_raw : right_edge_raw;
    int16 diff;

    // 搜索范围20~70行（十字路口通常在中上部）
    for (i = 70; i > 20 + PIXEL_OFFSET; i -= PIXEL_OFFSET) {
        top   = edge[i - PIXEL_OFFSET];
        under = edge[i];
        diff = (int16)top - (int16)under;

        if (is_left) {
            if (diff > 10) { *row = i; *col = under; return 1; }    // 左线向右跳变(入十字)
        } else {
            if (diff < -10) { *row = i; *col = under; return 1; }   // 右线向左跳变(入十字)
        }
    }
    return 0;
}

/**
 * @brief  上拐点检测：边线从外侧回归（从贴边状态跳回赛道内部）
 * @note   搜索100→15+2*PIXEL_OFFSET，跳变≥10，上方12行直线+下方16行至少2贴边点
 */
static uint8 Check_High_Turn(uint8 is_left, uint8 *row, uint8 *col) {
    uint8 i;
    uint8 *edge = is_left ? left_edge_raw : right_edge_raw;
    int16 diff;
    uint8 top, under;
    uint8 r0;

    // 从上往下搜索上拐点
    for (i = 100; i > 15 + 2*PIXEL_OFFSET; i -= 2*PIXEL_OFFSET) {
        top   = edge[i - 2*PIXEL_OFFSET];
        under = edge[i];
        // 左环：top - under ≥ 10（从边界跳回内部，差值正）
        // 右环：under - top ≥ 10（从边界跳回内部，差值正）
        diff = is_left ? (top - under) : (under - top);
        if (diff >= 10) {
            r0 = i - 2*PIXEL_OFFSET;   // 拐点行
            if (r0 >= 20) {
                // 上方12行须为直线
                if (IsLineStraight(edge, r0, r0 - 12)) {
                    // 下方16行须至少有2个贴边点（确认从外侧贴边状态跳回）
                    {
                        uint8 lost_cnt = 0;
                        uint8 k;
                        for (k = r0 + PIXEL_OFFSET; k <= r0 + 16; k += PIXEL_OFFSET) {
                            if (is_left ? (edge[k] <= 5) : (edge[k] >= SEARCH_IMAGE_W - 5))
                                lost_cnt++;
                        }
                        if (lost_cnt >= 2) {
                            *row = r0;
                            *col = top;
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

/**
 * @brief  出口拐点检测：对侧边线斜线→丢线转折点
 * @note   搜索110→50，非贴边→上找贴边行→中间斜率一致性(差≤6)
 */
static uint8 Check_Exit_Turn(uint8 is_left, uint8 *row, uint8 *col) {
    uint8 i, j;
    uint8 *edge = is_left ? left_edge_raw : right_edge_raw;
    uint8 boundary = is_left ? 5 : (SEARCH_IMAGE_W - 5);
    int16 first_diff;
    uint8 valid;

    // 从下往上搜索非贴边点（斜线段的末端）
    for (i = 110; i > 50; i -= PIXEL_OFFSET) {
        if (is_left ? (edge[i] <= boundary) : (edge[i] >= boundary)) continue;  // 跳过贴边点

        // 从该点继续向上找最近的贴边点（斜线段的起点）
        for (j = i - PIXEL_OFFSET; j > 20; j -= PIXEL_OFFSET) {
            if (is_left ? (edge[j] <= boundary) : (edge[j] >= boundary)) break;
        }
        if (j <= 20) continue;   // 未找到贴边起点

        // 验证斜线段斜率一致性
        valid = 1;
        first_diff = 0;
        {
            uint8 k;
            for (k = i; k > j + PIXEL_OFFSET; k -= PIXEL_OFFSET) {
                int16 cur_diff = (int16)edge[k - PIXEL_OFFSET] - (int16)edge[k];
                if (first_diff == 0) {
                    first_diff = cur_diff;
                } else if (abs(cur_diff - first_diff) > 6) {  // 斜率差>6 → 不是规则的斜线
                    valid = 0;
                    break;
                }
            }
        }
        if (valid) {
            *row = i;          // 出口拐点行
            *col = edge[i];    // 出口拐点列
            return 1;
        }
    }
    return 0;
}

/**
 * @brief  圆环主状态机
 * @note   状态0(空闲): 单侧丢线+下拐→状态1
 *         状态1(下拐确认): 等上拐, 200帧超时/10000陀螺仪超时复位
 *         状态2(入环补线): 陀螺仪≥3700→状态3; 画补偿线, 换参考列+差速系数
 *         状态3(等待出口): 陀螺仪≥22000→状态4
 *         状态4(找出口): 锁出口拐点→补线→陀螺仪≥阈值/双线恢复直线/33000超时→复位+400帧冷却
 *         关键阈值: 下拐差值>6, 上拐差值≥10, 陀螺仪3700/22000/33000, 冷却400帧
 */
void Ring(void) {
    uint8 left_con, right_con;
    uint8 is_left;
    uint8 temp_row, temp_col;
    uint8 other_straight;

    // ---- 预处理：检测左右边线连续性 ----
    left_con = isContinueLine(left_edge_line, 119, 30);
    right_con = isContinueLine(right_edge_line, 119, 30);

    // 状态0和1时检测十字路口（排除误判）
    if (ring_state == 0 || ring_state == 1) {
        Check_Crossroad_Guard();
    }

    // 空闲时双线正常+直线 → 快速复位（防止残留状态）
    if (ring_state == 0 && ring_cooldown == 0) {
        if (left_con && right_con &&
            IsLineStraight(left_edge_line, 80, 15) &&
            IsLineStraight(right_edge_line, 80, 15)) {
            Ring_Init();
            return;
        }
    }

    switch (ring_state) {

    // ========== 状态0：空闲，检测入环条件 ==========
    case 0:
        // 冷却中：减计数
        if (ring_cooldown > 0) {
            ring_cooldown--;
            break;
        }

        // 必须恰好一侧连续、一侧不连续（单侧丢线=环岛入口特征）
        if (!left_con && !right_con) break;   // 双侧都断 → 非环岛

        if (!left_con && right_con) is_left = 1;   // 左断右连 → 左环
        else if (left_con && !right_con) is_left = 0;   // 左连右断 → 右环
        else break;   // 双侧都连 → 非环岛

        // 额外校验：对侧丢线率不能太高
        if (is_left) {
            if (Edge_Lost_Ratio(0) > 50) break;             // 右侧丢线率>50% → 非环岛
            if (!isContinueLine(right_edge_line, 80, 20)) break;  // 右侧连续段不够长
        } else {
            if (Edge_Lost_Ratio(1) > 50) break;
            if (!isContinueLine(left_edge_line, 80, 20)) break;
        }

        // 检测下拐点（边线向外跳变）
        if (Check_Low_Turn(is_left, &temp_row, &temp_col, 0)) {
            // 对侧80-15行须为直线（确认对侧正常）
            other_straight = is_left ? IsLineStraight(right_edge_line, 80, 15) : IsLineStraight(left_edge_line, 80, 15);
            if (!other_straight) break;

            // 确认入环 → 进入状态1
            low_turn_confirmed = 1;
            low_turn_row = temp_row;
            low_turn_col = temp_col;
            state1_timeout = 0;
            gyro_integrating = 1;      // 开始陀螺仪积分
            gyro_accum = 0;
            ring_state = 1;

            // 记录环岛方向
            if (is_left) { ring_l = 1; ring_r = 0; debug_ring_dir = 1; }
            else         { ring_r = 1; ring_l = 0; debug_ring_dir = 2; }
        }
        break;

    // ========== 状态1：下拐已确认，等待上拐点 ==========
    case 1:
        if (state1_timeout >= 200) {   // 200帧超时 → 误判，复位
            Ring_Init();
            break;
        }
        state1_timeout++;

        if (!low_turn_confirmed) { ring_state = 0; break; }

        // 再次检测下拐：如果出现对侧下拐（十字），退出状态1
        if (Check_Low_Turn(ring_l ? 1 : 0, &temp_row, &temp_col, 1)) break;

        // 对侧底部丢线≥50%时跳过直线校验
        {
            uint16 lost_cnt;
            uint8 skip_straight = 0;
            if (ring_l) lost_cnt = Count_Lost_Lines_Full(0);
            else        lost_cnt = Count_Lost_Lines_Full(1);
            if (lost_cnt >= (SEARCH_IMAGE_H * 50 / 100)) {
                skip_straight = 1;
            }
            if (!skip_straight) {
                if (ring_l) { if (!IsLineStraight(right_edge_line, 45, 15)) break; }
                else        { if (!IsLineStraight(left_edge_line,  45, 15)) break; }
            }
        }

        // 陀螺仪积分超10000 → 转够了但没找到上拐，复位
        if (gyro_accum >= 10000) {
            Ring_Init();
            break;
        }

        // 检测上拐点（边线从贴边回归赛道内部）
        if (Check_High_Turn(ring_l ? 1 : 0, &temp_row, &temp_col))
        {
            upper_turn_row = temp_row;      // 记录上拐行
            upper_base_col = temp_col;      // 记录上拐列

            // 画入环补偿线：从上拐点斜向延伸到环岛对侧
            Draw_Compensate_Line(upper_turn_row, upper_base_col, ring_l ? 0 : 1);

            // 开启补线模式
            ring_compensate = 1;
            ring_scan_dir = ring_l ? 2 : 1;

            // 替换参考列为环岛外侧（让车沿环岛外侧巡线）
            old_reference_col = reference_col;
            if (ring_l) reference_col = 14;                      // 左环→参考列偏左
            else        reference_col = SEARCH_IMAGE_W - 14;     // 右环→参考列偏右

            // 禁用前馈并替换差速系数（环岛需要更激进的差速）
            ring_disable_feedforward = 1;
            saved_diff_kp = diff_kp;
            diff_kp = g_ring_diff_kp;
            diff_kp_modified = 1;

            ring_state = 2;
            gyro_accum = 0;            // 重新开始积分
            gyro_integrating = 1;
        }
        break;

    // ========== 状态2：入环补线，等待转向完成 ==========
    case 2:
        // 保持前馈禁用和参考列固定
        ring_disable_feedforward = 1;
        if (ring_l) reference_col = 14;
        else        reference_col = SEARCH_IMAGE_W - 14;

        // 陀螺仪积分≥3700 → 转向量足够，进入状态3
        if (gyro_accum >= 3700) {
            // 恢复正常巡线：关闭补线、恢复前馈、恢复参考列和差速系数
            ring_scan_dir = 0;
            ring_compensate = 0;
            ring_disable_feedforward = 0;
            reference_col = old_reference_col;
            diff_kp = saved_diff_kp;
            diff_kp_modified = 0;
            ring_state = 3;
        }
        break;

    // ========== 状态3：等待出口，累积足够转角 ==========
    case 3:
        if (gyro_accum >= 22000) {   // 转角足够 → 开始寻找出口
            ring_state = 4;
            exit_turn_locked = 0;     // 未锁定出口拐点
        }
        break;

    // ========== 状态4：寻找出口拐点 → 出环 ==========
    case 4:
        if (!exit_turn_locked) {
            // 尝试检测出口拐点（对侧边线的斜线→丢线转折点）
            if (Check_Exit_Turn(ring_l ? 0 : 1, &exit_turn_row, &exit_turn_col)) {
                // 锁定出口拐点
                exit_turn_locked = 1;
                ring_exit_compensate = 1;   // 开启出口补线
                ring_compensate = 1;

                // 替换参考列
                ring_scan_dir = ring_l ? 2 : 1;
                old_reference_col = reference_col;
                if (ring_l) reference_col = 15;
                else        reference_col = SEARCH_IMAGE_W - 15;

                // 设置出口陀螺仪阈值：当前积分+3500
                exit_gyro_threshold = gyro_accum + 3500;

                // 画出环补偿线：从出口拐点斜向延伸到远侧边界
                Draw_Exit_Compensate_Line(exit_turn_row, exit_turn_col, ring_l ? 0 : 1);
            }
            else if (gyro_accum >= 33000) {   // 超时未找到出口 → 强制复位
                Ring_Init();
                ring_cooldown = 400;           // 400帧冷却
            }
            break;
        }

        // 三个退出条件（满足任一即复位）：
        // 1) 陀螺仪达到出口阈值
        if (gyro_accum >= exit_gyro_threshold) {
            Ring_Init();
            ring_cooldown = 400;
        }
        // 2) 双线恢复连续且为直线（视觉确认已出环）
        else if (left_con && right_con &&
                 IsLineStraight(left_edge_line, 80, 15) &&
                 IsLineStraight(right_edge_line, 80, 15)) {
            Ring_Init();
            ring_cooldown = 400;
        }
        // 3) 陀螺仪超33000绝对上限
        else if (gyro_accum >= 33000) {
            Ring_Init();
            ring_cooldown = 400;
        }
        break;
    }
}
