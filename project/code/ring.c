#include "ring.h"
#include "servo.h"
#include <stdlib.h>

#define FIRST_TO_NORMAL_CONFIRM_FRAMES 3u
#define ENTER_TO_FIRST_CONFIRM_FRAMES  3u
#define TURN_TO_IN_CONFIRM_FRAMES      3u
#define IN_TO_OUT_MIN_FRAMES           20u
#define IN_TO_OUT_CONFIRM_FRAMES       3u
#define RING_IN_LEFT_TOP_COL           57u
#define RING_OUT_LEFT_TOP_COL          30u
#define RING_MIRROR_COL(col)           ((uint8)(SEARCH_IMAGE_W - 1u - (col)))
#define RING_IN_RIGHT_TOP_COL          130u
#define RING_OUT_RIGHT_TOP_COL         157u
#define RIGHT_IN_AUTO_OUT_FRAMES       40u
#define BACK_AUTO_EXIT_FRAMES          200u
#define RIGHT_RING_ENABLE              1u

#define RING_STEP_PRE_MEET              0u
#define RING_STEP_FIRST_MEETING         1u
#define RING_STEP_ENTER                 2u
#define RING_STEP_TURN                  3u
#define RING_STEP_IN                    4u
#define RING_STEP_OUT                   5u
#define RING_STEP_STRAIGHTEN            6u
#define RING_STEP_OVER                  7u

uint8 left_control_line[SEARCH_IMAGE_H];
uint8 right_control_line[SEARCH_IMAGE_H];

uint8 current_step = 0;
uint8 key_anlysis1 = 0;
uint8 key_anlysis2 = 0;
uint8 key_anlysis3 = 0;

uint8 ring_preMeet_flag = 0;
uint8 ring_l = 0;
uint8 ring_r = 0;
uint8 first_meeting_flag = 0;
uint8 ring_enter_flag = 0;
uint8 ring_turn_flag = 0;
uint8 Out_flag = 0;
uint8 Straighten_flag = 0;
uint8 ring_over_flag = 0;

static uint8 mid_under_flag = 0;
static uint8 first_to_normal_cnt = 0;
static uint8 enter_to_first_cnt = 0;
static uint8 turn_to_in_cnt = 0;
static uint8 in_to_out_cnt = 0;
static uint16 cnt_over = 0;
static uint16 back_frame_cnt = 0;

static void Ring_First_meeting_Right(void);
static void Ring_Enter_Right(void);
static void Ring_Turing_Right(void);
static void Ring_Ring_Ring_Right(void);
static void Ring_Out_Right(void);
static void Ring_Straighten_Right(void);
static void Ring_Change_Step(uint8 next_step);
void Ring_Over(void);

/*
 * 集中处理状态切换时需要清除的标志和计数器。
 * 左右处理函数只负责本阶段识别与补线，不再各自修改一堆状态变量。
 */
static void Ring_Change_Step(uint8 next_step)
{
    switch (next_step) {
    case RING_STEP_PRE_MEET:
        Ring_Over();
        return;

    case RING_STEP_FIRST_MEETING:
        first_meeting_flag = 0;
        ring_enter_flag = 0;
        mid_under_flag = 0;
        first_to_normal_cnt = 0;
        enter_to_first_cnt = 0;
        break;

    case RING_STEP_ENTER:
        ring_enter_flag = 0;
        mid_under_flag = 0;
        enter_to_first_cnt = 0;
        break;

    case RING_STEP_TURN:
        ring_turn_flag = 0;
        enter_to_first_cnt = 0;
        turn_to_in_cnt = 0;
        break;

    case RING_STEP_IN:
        Out_flag = 0;
        cnt_over = 0;
        in_to_out_cnt = 0;
        turn_to_in_cnt = 0;
        break;

    case RING_STEP_OUT:
        Straighten_flag = 0;
        in_to_out_cnt = 0;
        break;

    case RING_STEP_STRAIGHTEN:
        ring_over_flag = 0;
        back_frame_cnt = 0;
        break;

    case RING_STEP_OVER:
        break;

    default:
        Ring_Over();
        return;
    }

    current_step = next_step;
}

/* 计算两个无符号 8 位数的差值绝对值。 */
static uint8 abs_diff_u8(uint8 a, uint8 b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint8 Ring_Row_Width_Over(uint8 row, uint8 width)
{
    if (right_edge_line[row] <= left_edge_line[row]) {
        return 0;
    }

    return ((uint8)(right_edge_line[row] - left_edge_line[row]) > width) ? 1u : 0u;
}

static uint8 Ring_Left_Side_Ready(void)
{
    return (left_edge_line[110] >= 10u) ? 1 : 0;
}

static uint8 Ring_Right_Side_Ready(void)
{
    return (right_edge_line[110] <= RING_MIRROR_COL(10u)) ? 1 : 0;
}

/* 把列坐标限制在图像宽度范围内。 */
uint8 clamp_col_int16(int16 val)
{
    if (val < 0) return 0;
    if (val >= SEARCH_IMAGE_W) return (SEARCH_IMAGE_W - 1);
    return (uint8)val;
}

static void Ring_Fill_Line(uint8 start_row, uint8 end_row,
                           int16 start_col, int16 end_col, uint8 to_flag)
{
    int16 diff;
    int16 dir;
    int16 den;
    int16 base_step;
    int16 rem;
    int16 acc;
    int16 col;
    uint8 i;

    if (end_row <= start_row) {
        return;
    }

    den = (int16)(end_row - start_row);

    diff = end_col - start_col;
    dir = 1;
    if (diff < 0) {
        diff = (int16)(-diff);
        dir = -1;
    }

    base_step = diff / den;
    rem = (int16)(diff - base_step * den);
    acc = den >> 1;
    col = start_col;

    for (i = start_row + 1; i < end_row; i++) {
        col = (int16)(col + dir * base_step);
        acc = (int16)(acc + rem);
        if (acc >= den) {
            col = (int16)(col + dir);
            acc = (int16)(acc - den);
        }

        if (!to_flag) {
            left_control_line[i] = clamp_col_int16(col);
        } else {
            right_control_line[i] = clamp_col_int16(col);
        }
    }
}

/*
 * 圆环状态机入口。
 * 左右分发只在这里完成一次，避免先调用“通用函数”，再在函数内部二次跳转到右环函数。
 */
void Ring(void)
{
    /* 除预判阶段外，必须且只能锁定一个圆环方向。 */
    if (current_step != RING_STEP_PRE_MEET) {
        if (((ring_l != 0u) + (ring_r != 0u)) != 1u) {
            Ring_Over();
            return;
        }
    }

    switch (current_step) {
    case RING_STEP_PRE_MEET:
        if (Ring_Pre_Meet()) {
            Ring_Change_Step(RING_STEP_FIRST_MEETING);
        }
        break;

    case RING_STEP_FIRST_MEETING:
        if (ring_r) {
            Ring_First_meeting_Right();
        } else {
            Ring_First_meeting();
        }

        if (first_meeting_flag) {
            Ring_Change_Step(RING_STEP_ENTER);
        }
        break;

    case RING_STEP_ENTER:
        if (ring_r) {
            Ring_Enter_Right();
        } else {
            Ring_Enter();
        }

        if (ring_enter_flag) {
            Ring_Change_Step(RING_STEP_TURN);
        }
        break;

    case RING_STEP_TURN:
        if (ring_r) {
            Ring_Turing_Right();
        } else {
            Ring_Turing();
        }

        if (ring_turn_flag) {
            if (++turn_to_in_cnt >= TURN_TO_IN_CONFIRM_FRAMES) {
                Ring_Change_Step(RING_STEP_IN);
            }
            ring_turn_flag = 0;
        } else {
            turn_to_in_cnt = 0;
        }
        break;

    case RING_STEP_IN:
        /* 右环进入 IN 后只会直接调用一次该函数。 */
        if (ring_r) {
            Ring_Ring_Ring_Right();
        } else {
            Ring_Ring_Ring();
        }

        if (Out_flag) {
            Ring_Change_Step(RING_STEP_OUT);
        }
        break;

    case RING_STEP_OUT:
        if (ring_r) {
            Ring_Out_Right();
        } else {
            Ring_Out();
        }

        if (Straighten_flag) {
            Ring_Change_Step(RING_STEP_STRAIGHTEN);
        }
        break;

    case RING_STEP_STRAIGHTEN:
        if (back_frame_cnt < BACK_AUTO_EXIT_FRAMES) {
            back_frame_cnt++;
        }

        if (ring_r) {
            Ring_Straighten_Right();
        } else {
            Ring_Straighten();
        }

        if (ring_over_flag || back_frame_cnt >= BACK_AUTO_EXIT_FRAMES) {
            Ring_Change_Step(RING_STEP_OVER);
        }
        break;

    case RING_STEP_OVER:
    default:
        Ring_Over();
        break;
    }
}

/* 在左右控制线之间补一条斜线，用于圆环阶段修正缺失边线。 */
void draw_line(uint8 left_point, uint8 right_point, uint8 to_flag)
{
    uint8 start_row;
    uint8 end_row;
    int16 start_col;
    int16 end_col;

    if (left_point >= SEARCH_IMAGE_H) left_point = SEARCH_IMAGE_H - 1;
    if (right_point >= SEARCH_IMAGE_H) right_point = SEARCH_IMAGE_H - 1;

    if (left_point > right_point) {
        start_row = right_point;
        end_row = left_point;
        start_col = right_control_line[right_point];
        end_col = left_control_line[left_point];
    } else {
        start_row = left_point;
        end_row = right_point;
        start_col = left_control_line[left_point];
        end_col = right_control_line[right_point];
    }

    if (end_row == start_row) {
        return;
    }

    if (!to_flag) {
        Ring_Fill_Line(start_row, end_row, start_col, end_col, 0);
    } else {
        Ring_Fill_Line(start_row, end_row, start_col, end_col, 1);
    }
}

/* 连接同一侧控制线上的上下两个点，补齐中间缺失行。 */
void connect_point(uint8 under, uint8 top, uint8 l_r)
{
    int16 start_col;
    int16 end_col;

    if (under >= SEARCH_IMAGE_H) under = SEARCH_IMAGE_H - 1;
    if (top >= SEARCH_IMAGE_H) top = SEARCH_IMAGE_H - 1;
    if (under <= top) return;

    if (!l_r) {
        start_col = left_control_line[top];
        end_col = left_control_line[under];
        Ring_Fill_Line(top, under, start_col, end_col, 0);
    } else {
        start_col = right_control_line[top];
        end_col = right_control_line[under];
        Ring_Fill_Line(top, under, start_col, end_col, 1);
    }
}

/* 判断一条边线是否连续，突变过大则认为当前边线不稳定。 */
uint8 isContinueLine(uint8 *arr)
{
    int16 i;
    uint8 curr;
    uint8 prev;

    for (i = SEARCH_IMAGE_H - 1 - 4; i > 7 * 4; i -= 4) {
        curr = arr[i];
        prev = arr[i - 4];

        if (abs_diff_u8(curr, prev) > 20) {
            return 0;
        }
    }

    return 1;
}

/* 圆环预判：左右边线一侧连续、一侧不连续时认为可能进入圆环。 */
uint8 Ring_Pre_Meet(void)
{
    uint8 r_con;
    uint8 l_con;
 
    r_con = isContinueLine(right_edge_line);
    l_con = isContinueLine(left_edge_line);

    if (!l_con && r_con && Ring_Left_Side_Ready()) {
        ring_preMeet_flag = 1;
        ring_l = 1;
        ring_r = 0;
        return 1;
    }

#if RIGHT_RING_ENABLE
    if (!r_con && l_con && Ring_Right_Side_Ready()) {
        ring_preMeet_flag = 1;
        ring_l = 0;
        ring_r = 1;
        return 1;
    }
#endif

    ring_preMeet_flag = 0;
    ring_l = 0;
    ring_r = 0;
    return 0;
}

/* 只返回圆环预判结果，不直接修改主状态标志。 */
uint8 Ring_Pre_Meet_use(void)
{
    uint8 r_con;
    uint8 l_con;

    r_con = isContinueLine(right_edge_line);
    l_con = isContinueLine(left_edge_line);

    if (ring_l) {
        return (!l_con && r_con && Ring_Left_Side_Ready()) ? 1 : 0;
    }

#if RIGHT_RING_ENABLE
    if (ring_r) {
        return (!r_con && l_con && Ring_Right_Side_Ready()) ? 1 : 0;
    }
#endif

    return (r_con != l_con) ? 1 : 0;
}

static void Ring_First_To_Normal_Check(void)
{
    if (!Ring_Pre_Meet_use()) {
        if (++first_to_normal_cnt >= FIRST_TO_NORMAL_CONFIRM_FRAMES) {
            Ring_Change_Step(RING_STEP_PRE_MEET);
        }
    } else {
        first_to_normal_cnt = 0;
    }
}

/* 第一次遇到圆环边线变化时，寻找拐点并尝试补左边线。 */
void Ring_First_meeting(void)
{
    uint8 i;
    uint8 under;
    uint8 top;
    uint8 mid;
    uint8 turnPoint = 0;
    uint8 midPoint = 0;

    

    for (i = SEARCH_IMAGE_H - 1 - PIXEL_OFFSET; i > 30; i -= PIXEL_OFFSET) {
        under = left_edge_line[i];
        top = left_edge_line[i - PIXEL_OFFSET];

        if ((int16)under - (int16)top >= 15) {
            turnPoint = i;
            break;
        }
    }

    if (turnPoint == 0) {
        Ring_First_To_Normal_Check();
        return;
    }

    for (i = turnPoint - 4; i > 4; i -= 4) {
        under = left_edge_line[i + 4];
        mid = left_edge_line[i];
        top = left_edge_line[i - 4];

        if (mid >= under && mid >= top && mid > 30) {
            midPoint = i;
        }
    }

    if (midPoint) {
        connect_point(turnPoint, midPoint, 0);
    }

    if (midPoint &&
        Ring_Left_Side_Ready() &&
        left_edge_line[94] <= 2 &&
        left_edge_line[85] <= 2 &&
        (Ring_Row_Width_Over(55, 110u) ||
         Ring_Row_Width_Over(30, 110u) ||
         Ring_Row_Width_Over(35, 110u) ||
         Ring_Row_Width_Over(40, 110u))) {
        first_meeting_flag = 1;
        first_to_normal_cnt = 0;
        return;
    }

    Ring_First_To_Normal_Check();
}

/* 进入圆环阶段，持续补左边线，直到拐点移动到远端。 */
void Ring_Enter(void)
{
    uint8 i;
    uint8 under;
    uint8 mid;
    uint8 top;
    uint8 ttop;
    uint8 uunder;
    uint8 midPoint = 0;
		uint8 turn_flag = 0;

    for (i = SEARCH_IMAGE_H - 1 - 2 * 2; i > 20; i -= 2) {
        uunder = left_edge_line[i + 4 ];
        under  = left_edge_line[i + 2];
        mid    = left_edge_line[i];
        top    = left_edge_line[i - 2];
        ttop   = left_edge_line[i - 4];

        if (mid < 8) {
            continue;
        }

        if (mid >= under && (mid - under) < 20 &&
            under >= uunder && (under - uunder) < 20 &&
            mid >= top && (mid - top) < 20 &&
            top >= ttop && (top - ttop) < 20 &&
            mid > 20) {
            midPoint = i;
            break;
        }
    }

    if (midPoint) {
        enter_to_first_cnt = 0;
        connect_point(SEARCH_IMAGE_H - 1, midPoint, 0);
    } else {
        if (++enter_to_first_cnt >= ENTER_TO_FIRST_CONFIRM_FRAMES) {
            Ring_Change_Step(RING_STEP_FIRST_MEETING);
        }
        return;
    }

    if (midPoint >= 60) {
        mid_under_flag = 1;
    }
		for (i = 81; i > 10; i -= 2) {
        under = left_edge_line[i];
        top = left_edge_line[i - 2];
				
        if (top - under >= 15 && left_edge_line[110] >= 10 && left_edge_line[i - 4] > 20 &&left_edge_line[i-6] > 20) {
            turn_flag = 1;
					break;
        }
    }
    if (mid_under_flag && (turn_flag || midPoint < 45)) {
        ring_enter_flag = 1;
        enter_to_first_cnt = 0;
    }
}

/* 圆环转向阶段，找到左上拐点后用右控制线辅助车辆转入。 */
void Ring_Turing(void)
{
    uint8 i;
    uint8 under;
    uint8 top;
    uint8 leftTopPoint = 0;

    for (i = SEARCH_IMAGE_H - 1; i > 10; i -= 2) {
        top = left_edge_line[i - 2];
        under = left_edge_line[i];

        if (top- under >= 20 && left_edge_line[i-4] > 20 && left_edge_line[i-6] >20) {
            leftTopPoint = i - 2;
            break;
        }
    }

    if (leftTopPoint == 0) {
        return;
    }

    draw_line(leftTopPoint, SEARCH_IMAGE_H - 1, 1);

    if (leftTopPoint >= controlReferenceLine - 30) {
        ring_turn_flag = 1;
    }
}

/* 车身处在圆环内部时，修正右控制线并判断是否接近出口。 */
void Ring_Ring_Ring(void)
{
    uint8 i;
    uint8 minPoint = SEARCH_IMAGE_H - 1;
    uint8 index;
    uint8 minn = SEARCH_IMAGE_W - 1;

    if (cnt_over < IN_TO_OUT_MIN_FRAMES) {
        cnt_over++;
    }

    if (left_control_line[controlReferenceLine] > Mid_Col) {
        left_control_line[controlReferenceLine] = 0;
    }

    for (i = SEARCH_IMAGE_H - 1; i > PIXEL_OFFSET * 3; i -= PIXEL_OFFSET) {
        index = right_edge_line[i];
        if (index < minn) {
            minn = index;
            minPoint = i;
        }
    }

    if (minPoint > PIXEL_OFFSET &&
        minPoint < SEARCH_IMAGE_H - PIXEL_OFFSET &&
        (((int16)right_edge_line[minPoint - PIXEL_OFFSET] - (int16)right_edge_line[minPoint] < 30 &&
         (int16)right_edge_line[minPoint + PIXEL_OFFSET] - (int16)right_edge_line[minPoint] < 30) ||
        minPoint < 40)) {
        left_control_line[0] = RING_IN_LEFT_TOP_COL;
        right_control_line[0] = RING_IN_LEFT_TOP_COL;
        draw_line(0, SEARCH_IMAGE_H - 1, 1);
    }

    if (cnt_over >= IN_TO_OUT_MIN_FRAMES &&
        right_edge_line[50] > 180 &&
        right_edge_line[60] > 180 &&

        left_edge_line[50] < 2) {
        if (++in_to_out_cnt >= IN_TO_OUT_CONFIRM_FRAMES) {
            Out_flag = 1;
            in_to_out_cnt = 0;
        }
    } else {
        in_to_out_cnt = 0;
    }
}

/* 出圆环阶段，继续补右控制线并判断是否可以开始回正。 */
void Ring_Out(void)
{
    left_control_line[0] = RING_OUT_LEFT_TOP_COL;
    right_control_line[0] = RING_OUT_LEFT_TOP_COL;
    draw_line(0, SEARCH_IMAGE_H - 1, 1);

    if ((left_edge_line[30] > 16 ||
         left_edge_line[40] > 16 ||
         left_edge_line[50] > 16) &&
        right_edge_line[40] < 170 &&
        right_edge_line[50] < 170 &&
        right_edge_line[60] < 170 &&
				right_edge_line[45] < right_edge_line[50] &&
				right_edge_line[40] < right_edge_line[45] &&
				right_edge_line[50] < right_edge_line[55]
		)
        Straighten_flag = 1;
}

/* 回正阶段，重新补左线，等左右边线恢复连续后结束圆环。 */
void Ring_Straighten(void)
{
    uint8 i;
    uint8 under;
    uint8 mid;
    uint8 top;
    uint8 midPoint = 0;

    for (i = SEARCH_IMAGE_H - 1 - PIXEL_OFFSET; i > PIXEL_OFFSET; i -= PIXEL_OFFSET) {
        under = left_edge_line[i + PIXEL_OFFSET];
        mid = left_edge_line[i];
        top = left_edge_line[i - PIXEL_OFFSET];

        if (mid >= under && mid >= top && mid > 30) {
            midPoint = i;
        }
    }

    if (midPoint) {
        connect_point(SEARCH_IMAGE_H - 1, midPoint, 0);
    }

    if (isContinueLine(left_edge_line) && isContinueLine(right_edge_line) && left_edge_line[110] >= 5) {
        ring_over_flag = 1;
    }
} 

/* 圆环结束或异常复位时，清空所有圆环状态标志。 */
static void Ring_First_meeting_Right(void)
{
    uint8 i;
    uint8 under;
    uint8 top;
    uint8 mid;
    uint8 turnPoint = 0;
    uint8 midPoint = 0;

    for (i = SEARCH_IMAGE_H - 1 - PIXEL_OFFSET; i > 30; i -= PIXEL_OFFSET) {
        under = right_edge_line[i];
        top = right_edge_line[i - PIXEL_OFFSET];

        if ((int16)top - (int16)under >= 15) {
            turnPoint = i;
            break;
        }
    }

    if (turnPoint == 0) {
        Ring_First_To_Normal_Check();
        return;
    }

    for (i = turnPoint - 4; i > 4; i -= 4) {
        under = right_edge_line[i + 4];
        mid = right_edge_line[i];
        top = right_edge_line[i - 4];

        if (mid <= under && mid <= top && mid < RING_MIRROR_COL(30u)) {
            midPoint = i;
        }
    }

    if (midPoint) {
        connect_point(turnPoint, midPoint, 1);
    }

    if (midPoint &&
        Ring_Right_Side_Ready() &&
        right_edge_line[94] >= RING_MIRROR_COL(2u) &&
        right_edge_line[85] >= RING_MIRROR_COL(2u) &&
        (Ring_Row_Width_Over(55, 110u) ||
         Ring_Row_Width_Over(30, 110u) ||
         Ring_Row_Width_Over(35, 110u) ||
         Ring_Row_Width_Over(40, 110u))) {
        first_meeting_flag = 1;
        first_to_normal_cnt = 0;
        return;
    }

    Ring_First_To_Normal_Check();
}

static void Ring_Enter_Right(void)
{
    uint8 i;
    uint8 under;
    uint8 mid;
    uint8 top;
    uint8 ttop;
    uint8 uunder;
    uint8 midPoint = 0;
    uint8 turn_flag = 0;

    for (i = SEARCH_IMAGE_H - 1 - 2 * 2; i > 20; i -= 2) {
        uunder = right_edge_line[i + 4];
        under  = right_edge_line[i + 2];
        mid    = right_edge_line[i];
        top    = right_edge_line[i - 2];
        ttop   = right_edge_line[i - 4];

        if (mid > RING_MIRROR_COL(8u)) {
            continue;
        }

        if (mid <= under && (under - mid) < 20 &&
            under <= uunder && (uunder - under) < 20 &&
            mid <= top && (top - mid) < 20 &&
            top <= ttop && (ttop - top) < 20 &&
            mid < RING_MIRROR_COL(20u)) {
            midPoint = i;
            break;
        }
    }

    if (midPoint) {
        enter_to_first_cnt = 0;
        connect_point(SEARCH_IMAGE_H - 1, midPoint, 1);
    } else {
        if (++enter_to_first_cnt >= ENTER_TO_FIRST_CONFIRM_FRAMES) {
            Ring_Change_Step(RING_STEP_FIRST_MEETING);
        }
        return;
    }

    if (midPoint >= 60) {
        mid_under_flag = 1;
    }

    for (i = 81; i > 10; i -= 2) {
        under = right_edge_line[i];
        top = right_edge_line[i - 2];

        if ((int16)under - (int16)top >= 15 &&
            right_edge_line[110] <= RING_MIRROR_COL(10u) &&
            right_edge_line[i - 4] < RING_MIRROR_COL(20u) &&
            right_edge_line[i - 6] < RING_MIRROR_COL(20u)) {
            turn_flag = 1;
            break;
        }
    }

    if (mid_under_flag && (turn_flag || midPoint < 45)) {
        ring_enter_flag = 1;
        enter_to_first_cnt = 0;
    }
}

static void Ring_Turing_Right(void)
{
    uint8 i;
    uint8 under;
    uint8 top;
    uint8 rightTopPoint = 0;

    for (i = SEARCH_IMAGE_H - 1; i > 10; i -= 2) {
        top = right_edge_line[i - 2];
        under = right_edge_line[i];

        if ((int16)under - (int16)top >= 20 &&
            right_edge_line[i - 4] < RING_MIRROR_COL(20u) &&
            right_edge_line[i - 6] < RING_MIRROR_COL(20u)) {
            rightTopPoint = i - 2;
            break;
        }
    }

    if (rightTopPoint == 0) {
        return;
    }

    draw_line(SEARCH_IMAGE_H - 1, rightTopPoint, 0);

    if (rightTopPoint >= controlReferenceLine - 30) {
        ring_turn_flag = 1;
    }
}

static void Ring_Ring_Ring_Right(void)
{
    if (cnt_over < RIGHT_IN_AUTO_OUT_FRAMES) {
        cnt_over++;
    }

    left_control_line[SEARCH_IMAGE_H - 1] = 0;
    left_control_line[0] = RING_IN_RIGHT_TOP_COL;
    right_control_line[0] = RING_IN_RIGHT_TOP_COL;
    draw_line(SEARCH_IMAGE_H - 1, 0, 0);

    if (cnt_over >= RIGHT_IN_AUTO_OUT_FRAMES) {
        Out_flag = 1;
    }
}

static void Ring_Out_Right(void)
{
    left_control_line[SEARCH_IMAGE_H - 1] = 0;
    left_control_line[0] = RING_OUT_RIGHT_TOP_COL;
    right_control_line[0] = RING_OUT_RIGHT_TOP_COL;
    draw_line(SEARCH_IMAGE_H - 1, 0, 0);

    if ((right_edge_line[30] < RING_MIRROR_COL(16u) ||
         right_edge_line[40] < RING_MIRROR_COL(16u) ||
         right_edge_line[50] < RING_MIRROR_COL(16u)) &&
        left_edge_line[40] > RING_MIRROR_COL(170u) &&
        left_edge_line[50] > RING_MIRROR_COL(170u) &&
        left_edge_line[60] > RING_MIRROR_COL(170u) &&
        left_edge_line[45] > left_edge_line[50] &&
        left_edge_line[40] > left_edge_line[45] &&
        left_edge_line[50] > left_edge_line[55]) {
        Straighten_flag = 1;
    }
}

static void Ring_Straighten_Right(void)
{
    uint8 i;
    uint8 under;
    uint8 mid;
    uint8 top;
    uint8 midPoint = 0;

    for (i = SEARCH_IMAGE_H - 1 - PIXEL_OFFSET; i > PIXEL_OFFSET; i -= PIXEL_OFFSET) {
        under = right_edge_line[i + PIXEL_OFFSET];
        mid = right_edge_line[i];
        top = right_edge_line[i - PIXEL_OFFSET];

        if (mid <= under && mid <= top && mid < RING_MIRROR_COL(30u)) {
            midPoint = i;
        }
    }

    if (midPoint) {
        connect_point(SEARCH_IMAGE_H - 1, midPoint, 1);
    }

    if (isContinueLine(left_edge_line) && isContinueLine(right_edge_line) &&
        right_edge_line[110] <= RING_MIRROR_COL(5u)) {
        ring_over_flag = 1;
    }
}

void Ring_Over(void)
{
    ring_preMeet_flag = 0;
    first_meeting_flag = 0;
    ring_l = 0;
    ring_r = 0;
    ring_enter_flag = 0;
    ring_turn_flag = 0;
    Out_flag = 0;
    Straighten_flag = 0;
    ring_over_flag = 0;
    mid_under_flag = 0;
    first_to_normal_cnt = 0;
    enter_to_first_cnt = 0;
    turn_to_in_cnt = 0;
    in_to_out_cnt = 0;
    current_step = 0;
    cnt_over = 0;
    back_frame_cnt = 0;
}
