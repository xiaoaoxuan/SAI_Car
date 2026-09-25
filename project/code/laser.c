/**
 * @file    laser.c
 * @brief   激光打靶模块：9路激光控制、靶子四边界检测(Target_find)、打靶状态机(Aim)
 * @author  RRZ
 * @date    2026-05-16
 * @note    这个打靶仅为测试版！！部分赛道可用，具体需要根据具体赛道调整！！！
 */

#include "Laser.h"
#include "image.h"

extern uint8 current_step;
extern int32 err_sum;
extern int16 imu660ra_gyro_z;
extern uint8 lost_left, lost_right;
extern int16 g_base_speed;

uint16 tar_i;
uint8  tar_flag=0;
uint8  tar_x;
uint8  tar_y;
uint8  tar_detected = 0;
uint8  tar_timeout = 0;

uint8 tar_th = 80;     // 靶环灰度阈值，低于此值视为黑色靶环
uint8 tar_laser = 0;

const gpio_pin_enum laser_pins[LASER_NUM] = {
    IO_P16, IO_P02, IO_P53, IO_P07, IO_P06, IO_P52, IO_P03, IO_P00, IO_P01
};

/**
 * @brief  9路激光GPIO初始化
 */
void laser_init(void)
{
    uint8 i;
    for (i = 0; i < LASER_NUM; i++)
        gpio_init(laser_pins[i], GPO, 0, GPO_PUSH_PULL);
}

/**
 * @brief  9路激光全开
 */
void laser_on(void)
{
    uint8 i;
    for (i = 1; i <= LASER_NUM; i++) light(i, 1);
}

/**
 * @brief  9路激光全关
 */
void laser_off(void)
{
    uint8 i;
    for (i = 1; i <= LASER_NUM; i++) light(i, 0);
}

/**
 * @brief  单路激光控制
 * @param  idx     激光编号 1~9
 * @param  on_off  1=亮, 0=灭
 */
void light(uint8 idx, uint8 on_off)
{
    if (idx < 1 || idx > LASER_NUM) return;
    gpio_set_level(laser_pins[idx - 1], on_off ? 1 : 0);
}

uint8 top_d=0,under_d=0,l_d=0,r_d=0;
uint8 debug_stage = 0;
uint8 tar_step_flags = 0;

#define Pre_Line   45     // 预扫描行
uint8 pre_find_offset=0;

/**
 * @brief  预扫描：在 Pre_Line 行检测暗色目标，计算偏移量辅助寻靶
 * @note   左右夹逼赛道内暗像素，宽度<8则无效
 */
void Pre_Scan(void){
    uint8 i,x,pre_l=0,pre_r=0;
    uint8 l_edge=left_edge_line[Pre_Line]+4;
    uint8 r_edge=right_edge_line[Pre_Line]-4;

    for(i=l_edge;i<r_edge;i+=2){
        x=mt9v03x_image[Pre_Line][i];
        if(x<tar_th){
            pre_l=i;
            break;
        }
    }
    for(i=r_edge;i<l_edge;i-=2){
        x=mt9v03x_image[Pre_Line][i];
        if(x<tar_th){
            pre_r=i;
            break;
        }
    }

    if(pre_r-pre_l<8 || pre_l > pre_r || pre_l == 0 || pre_r == 0){
        pre_find_offset=0;
        return ;
    }
    else {
        pre_find_offset=((pre_l+pre_r)>>1)-(SEARCH_IMAGE_W>>1);
    }
}

#define TopFind      40      // 靶子搜索上界行
#define BottomFind   110     // 靶子搜索下界行
#define MinTargetH    8      // 靶子最小高度
#define MaxTargetH   30      // 靶子最大高度
#define MinTargetW   10      // 靶子最小宽度
#define MaxTargetW   38      // 靶子最大宽度
#define FindOffset    3      // 搜索步长
#define JUMP_THRESHOLD 60    // 靶子边界灰度跳变阈值

/**
 * @brief  靶子检测核心：四步灰度跳变定位上/下/左/右边界，校验后映射激光编号
 * @note   步骤: 下边界(亮→暗)→上边界(暗→亮)→左边界→右边界→赛道内校验→57~131列映射1~9号激光
 */
void Target_find(void)
{
    uint8 i, j;
    uint8 tar_y_local;
    uint8 *row_ptr, *row_prev;
    uint8 left_bound, right_bound;
    uint8 under_col = 0;
    uint8 step1_ok = 0, step2_ok = 0, step3_ok = 0, step4_ok = 0;
    uint8 track_white;

    tar_flag = 0;
    tar_detected = 0;
    tar_step_flags = 0;
    top_d = 0;
    under_d = 0;
    l_d = 0;
    r_d = 0;

    {
        uint8 min_white = SEARCH_IMAGE_W, max_white = 0;
        for (i = 40; i <= BottomFind; i += 2)
        {
            for (j = 0; j < SEARCH_IMAGE_W; j++)
            {
                if (mt9v03x_image[i][j] < white_min_point)
                {
                    if (j < min_white) min_white = j;
                    break;
                }
            }
            for (j = SEARCH_IMAGE_W - 1; j > 0; j--)
            {
                if (mt9v03x_image[i][j] < white_min_point)
                {
                    if (j > max_white) max_white = j;
                    break;
                }
            }
        }
        if (min_white >= max_white) return;
        left_bound  = min_white;
        right_bound = max_white;
    }
    if (left_bound >= right_bound) return;
    if (right_bound > SEARCH_IMAGE_W - 1) return;

    {
        uint32 sum = 0;
        uint16 cnt = 0;
        uint8 r, c;
        for (r = SEARCH_IMAGE_H - 4; r < SEARCH_IMAGE_H; r++) {
            uint8 l = left_control_line[r];
            uint8 r_edge = right_control_line[r];
            if (l >= r_edge) continue;
            for (c = l; c <= r_edge; c++) {
                sum += mt9v03x_image[r][c];
                cnt++;
            }
        }
        track_white = (cnt > 0) ? (uint8)(sum / cnt) : 128;
    }

    /************* 1. 下边界：从下往上扫，亮→暗跳变 *************/
    for (i = BottomFind; i > TopFind + FindOffset; i -= FindOffset)
    {
        row_prev = mt9v03x_image[i - FindOffset];
        row_ptr  = mt9v03x_image[i];
        for (j = left_bound + 1; j < right_bound - 1; j += FindOffset)
        {
            if ((int16)row_ptr[j] - (int16)row_prev[j] > JUMP_THRESHOLD)
            {
                under_d = i - FindOffset;
                under_col = j;
                step1_ok = 1;
                break;
            }
        }
        if (step1_ok) break;
    }
    if (step1_ok) tar_step_flags++;

    /************* 2. 上边界：从下边界上方40行往下扫，亮→暗跳变 *************/
    if (step1_ok)
    {
        uint8 top_start = (under_d > 40) ? (under_d - 40) : TopFind;
        for (i = top_start + FindOffset; i < under_d; i += FindOffset)
        {
            row_prev = mt9v03x_image[i - FindOffset];
            row_ptr  = mt9v03x_image[i];
            for (j = left_bound; j < right_bound; j += FindOffset)
            {
                if ((int16)row_prev[j] - (int16)row_ptr[j] > JUMP_THRESHOLD)
                {
                    top_d = i;
                    step2_ok = 1;
                    break;
                }
            }
            if (step2_ok) break;
        }
    }
    if (step2_ok) tar_step_flags++;

    /************* 3. 左边界 *************/
    if (step1_ok && step2_ok && under_d > top_d)
    {
        uint8 row_start = (under_d > 60) ? (under_d - 60) : top_d;
        for (i = under_d; i > row_start; i -= 2)
        {
            row_ptr = mt9v03x_image[i];
            for (j = left_bound; j < under_col - FindOffset; j += FindOffset)
            {
                if ((int16)row_ptr[j] - (int16)row_ptr[j + FindOffset] > JUMP_THRESHOLD)
                {
                    l_d = j;
                    step3_ok = 1;
                    break;
                }
            }
            if (step3_ok) break;
        }
    }
    if (step3_ok) tar_step_flags++;

    /************* 4. 右边界 *************/
    if (step1_ok && step2_ok && under_d > top_d)
    {
        uint8 row_start = (under_d > 60) ? (under_d - 60) : top_d;
        for (i = under_d; i > row_start; i -= 2)
        {
            row_ptr = mt9v03x_image[i];
            for (j = right_bound; j > under_col + FindOffset; j -= FindOffset)
            {
                if ((int16)row_ptr[j] - (int16)row_ptr[j - FindOffset] > JUMP_THRESHOLD)
                {
                    r_d = j;
                    step4_ok = 1;
                    break;
                }
            }
            if (step4_ok) break;
        }
    }
    if (step4_ok) tar_step_flags++;

    /************* 校验 *************/
    {
        uint8 done = 0;
        uint8 width, height;
        uint8 missing_left  = step1_ok && step2_ok && !step3_ok && step4_ok;
        uint8 missing_right = step1_ok && step2_ok && step3_ok && !step4_ok;
        uint8 all_four      = step1_ok && step2_ok && step3_ok && step4_ok;

        if (all_four)
        {
            width  = r_d - l_d;
            height = under_d - top_d;

            if (width < 38 && height > 8)
            {
                tar_x = (l_d + r_d) >> 1;
                done = 1;
            }
            else if (width > 55)
            {
                if ((uint8)abs((int16)under_col - (int16)l_d) < (uint8)abs((int16)under_col - (int16)r_d))
                {
                    tar_x = l_d + 10;
                }
                else
                {
                    tar_x = r_d - 10;
                }
                done = 1;
            }
        }
        else if (missing_left || missing_right)
        {
            height = under_d - top_d;
            if (height < 32)
            {
                if (missing_left)  tar_x = r_d - 10;
                if (missing_right) tar_x = l_d + 10;
                done = 1;
            }
        }

        if (done)
        {
            {
                uint8 mid_row = (top_d + under_d) >> 1;
                uint8 in_track = 1;
                if (step3_ok) {
                    if (l_d < left_control_line[top_d] || l_d < left_control_line[under_d] || l_d < left_control_line[mid_row])
                        in_track = 0;
                }
                if (step4_ok) {
                    if (r_d > right_control_line[top_d] || r_d > right_control_line[under_d] || r_d > right_control_line[mid_row])
                        in_track = 0;
                }
                if (!in_track) done = 0;
            }

            if (tar_x <= 30 || tar_x >= SEARCH_IMAGE_W - 30)
                done = 0;
        }

        if (done)
        {
            tar_y = (top_d + under_d) >> 1;

            if (tar_x >= 57 && tar_x <= 131) {
                uint8 seg = (uint8)((uint16)(tar_x - 57) * 9 / 74);
                if (seg > 8) seg = 8;
                tar_laser = 1 + seg;
            }
            tar_flag = 1;
            tar_detected = 1;
            tar_step_flags = 5;
        }
    }
}

#define AIM_LASER_MS     15    // 激光点亮持续时间(ms)
#define AIM_COOLDOWN_MS  500   // 两次打靶冷却间隔(ms)

uint8  aim_step = 0;
uint8  shot_count = 0;
uint16 aim_timer = 0;
uint8  aim_laser = 0;
int16  g_target_col = Mid_Col;

/**
 * @brief  打靶主循环（主循环调用，每帧一次）
 * @note   非赛段时执行，检测到靶子且 under_d>=55 时点亮对应激光
 */
void Aim_Process_MainLoop(void)
{
    if (current_step >= 1 && current_step <= 6)
        return;

    Target_find();

    if (aim_step == 0)
    {
        if (tar_flag && under_d >= 55)
        {
            aim_step = 1;
            aim_timer = 0;
            aim_laser = tar_laser;
            shot_count++;
            light(aim_laser, 1);
        }
    }
}

/**
 * @brief  打靶中断处理（每5ms一次）
 * @note   aim_step=1: 点亮 AIM_LASER_MS ms 后关闭; aim_step=2: 冷却 AIM_COOLDOWN_MS ms 后回0
 */
void Aim_Process_Interrupt(void)
{
    if (aim_step == 1)
    {
        aim_timer += 5;
        if (aim_timer >= AIM_LASER_MS)
        {
            light(aim_laser, 0);
            aim_step = 2;
            aim_timer = 0;
        }
    }
    else if (aim_step == 2)
    {
        aim_timer += 5;
        if (aim_timer >= AIM_COOLDOWN_MS)
        {
            aim_step = 0;
            aim_timer = 0;
        }
    }
}

/**
 * @brief  取消打靶：关闭激光并复位状态机
 */
void Aim_Cancel(void)
{
    if (aim_step != 0) {
        light(aim_laser, 0);
    }
    aim_step = 0;
    aim_timer = 0;
}

uint8 laser_flow_enable = 1;

/**
 * @brief  9路激光循环流水灯（调试用，每100ms切换一路）
 */
void Laser_Flow_Process(void)
{
    static uint32 last_time = 0;
    static uint8  flow_idx  = 1;
    extern uint32 time;

    if (!laser_flow_enable) return;

    if (time - last_time >= 100)
    {
        last_time = time;
        laser_off();
        light(flow_idx, 1);
        flow_idx++;
        if (flow_idx > LASER_NUM) flow_idx = 1;
    }
}
