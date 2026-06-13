#include "ring.h"
#include "servo.h"
#include <stdlib.h>

#define FIRST_TO_NORMAL_CONFIRM_FRAMES 3u
#define ENTER_TO_FIRST_CONFIRM_FRAMES  3u
#define IN_TO_OUT_MIN_FRAMES           20u
#define IN_TO_OUT_CONFIRM_FRAMES       3u

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
static uint8 in_to_out_cnt = 0;
static uint16 cnt_over = 0;

/* 计算两个无符号 8 位数的差值绝对值。 */
static uint8 abs_diff_u8(uint8 a, uint8 b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* 把列坐标限制在图像宽度范围内。 */
uint8 clamp_col_int16(int16 val)
{
    if (val < 0) return 0;
    if (val >= SEARCH_IMAGE_W) return (SEARCH_IMAGE_W - 1);
    return (uint8)val;
}

/* 圆环状态机入口，每帧根据当前阶段调用对应处理函数。 */
void Ring(void)
{
    switch (current_step) {
    case 0:
        Ring_Pre_Meet();
        if (ring_preMeet_flag) {
            first_meeting_flag = 0;
            mid_under_flag = 0;
            first_to_normal_cnt = 0;
            current_step = 1;
        }
        break;

    case 1:
        Ring_First_meeting();
        if (first_meeting_flag) {
            ring_enter_flag = 0;
            mid_under_flag = 0;
            enter_to_first_cnt = 0;
            current_step = 2;
        }
        break;

    case 2:
        Ring_Enter();
        if (ring_enter_flag) {
            ring_turn_flag = 0;
            enter_to_first_cnt = 0;
            current_step = 3;
        }
        break;

    case 3:
        Ring_Turing();
        if (ring_turn_flag) {
            Out_flag = 0;
            cnt_over = 0;
            in_to_out_cnt = 0;
            current_step = 4;
        }
        break;

    case 4:
        Ring_Ring_Ring();
        if (Out_flag) {
            Straighten_flag = 0;
            in_to_out_cnt = 0;
            current_step = 5;
        }
        break;

    case 5:
        Ring_Out();
        if (Straighten_flag) {
            ring_over_flag = 0;
            current_step = 6;
        }
        break;

    case 6:
        Ring_Straighten();
        if (ring_over_flag) {
            current_step = 7;
        }
        break;

    case 7:
        Ring_Over();
        break;

    default:
        Ring_Over();
        break;
    }
}

/* 在左右控制线之间补一条斜线，用于圆环阶段修正缺失边线。 */
void draw_line(uint8 left_point, uint8 right_point, uint8 to_flag)
{
    uint8 i;
    uint8 start_row;
    uint8 end_row;
    int16 start_col;
    int16 end_col;
    int16 den;
    int32 num;
    int16 val;

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

    den = (int16)(end_row - start_row);

    if (!to_flag) {
        for (i = start_row + 1; i < end_row; i++) {
            num = (int32)(end_col - start_col) * (int32)(i - start_row);
            if (num >= 0) val = (int16)(start_col + (num + (den >> 1)) / den);
            else          val = (int16)(start_col + (num - (den >> 1)) / den);
            left_control_line[i] = clamp_col_int16(val);
        }
    } else {
        for (i = start_row + 1; i < end_row; i++) {
            num = (int32)(end_col - start_col) * (int32)(i - start_row);
            if (num >= 0) val = (int16)(start_col + (num + (den >> 1)) / den);
            else          val = (int16)(start_col + (num - (den >> 1)) / den);
            right_control_line[i] = clamp_col_int16(val);
        }
    }
}

/* 连接同一侧控制线上的上下两个点，补齐中间缺失行。 */
void connect_point(uint8 under, uint8 top, uint8 l_r)
{
    uint8 i;
    int16 start_col;
    int16 end_col;
    int16 den;
    int32 num;
    int16 val;

    if (under >= SEARCH_IMAGE_H) under = SEARCH_IMAGE_H - 1;
    if (top >= SEARCH_IMAGE_H) top = SEARCH_IMAGE_H - 1;
    if (under <= top) return;

    den = (int16)(under - top);

    if (!l_r) {
        start_col = left_control_line[top];
        end_col = left_control_line[under];
        for (i = top + 1; i < under; i++) {
            num = (int32)(end_col - start_col) * (int32)(i - top);
            if (num >= 0) val = (int16)(start_col + (num + (den >> 1)) / den);
            else          val = (int16)(start_col + (num - (den >> 1)) / den);
            left_control_line[i] = clamp_col_int16(val);
        }
    } else {
        start_col = right_control_line[top];
        end_col = right_control_line[under];
        for (i = top + 1; i < under; i++) {
            num = (int32)(end_col - start_col) * (int32)(i - top);
            if (num >= 0) val = (int16)(start_col + (num + (den >> 1)) / den);
            else          val = (int16)(start_col + (num - (den >> 1)) / den);
            right_control_line[i] = clamp_col_int16(val);
        }
    }
}

/* 判断一条边线是否连续，突变过大则认为当前边线不稳定。 */
uint8 isContinueLine(uint8 *arr)
{
    int16 i;
    uint8 curr;
    uint8 prev;

    for (i = SEARCH_IMAGE_H - 1 - PIXEL_OFFSET; i > 7 * PIXEL_OFFSET; i -= PIXEL_OFFSET) {
        curr = arr[i];
        prev = arr[i - PIXEL_OFFSET];

        if (abs_diff_u8(curr, prev) > 30) {
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

    if (r_con != l_con) {
        ring_preMeet_flag = 1;
        return 1;
    }

    ring_preMeet_flag = 0;
    return 0;
}

/* 只返回圆环预判结果，不直接修改主状态标志。 */
uint8 Ring_Pre_Meet_use(void)
{
    uint8 r_con;
    uint8 l_con;

    r_con = isContinueLine(right_edge_line);
    l_con = isContinueLine(left_edge_line);

    return (r_con != l_con) ? 1 : 0;
}

static void Ring_First_To_Normal_Check(void)
{
    if (!Ring_Pre_Meet_use()) {
        if (++first_to_normal_cnt >= FIRST_TO_NORMAL_CONFIRM_FRAMES) {
            current_step = 0;
            ring_preMeet_flag = 0;
            first_meeting_flag = 0;
            first_to_normal_cnt = 0;
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
        left_edge_line[100] <= 2 &&
        left_edge_line[85] <= 2 &&
				(
        left_edge_line[55] <= 20 ||
				left_edge_line[30] <= 20 ||
				left_edge_line[35] <= 20 ||
				left_edge_line[40] <= 20 
		
				)
				) {
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
            first_meeting_flag = 0;
            ring_enter_flag = 0;
            mid_under_flag = 0;
            enter_to_first_cnt = 0;
            current_step = 1;
        }
        return;
    }

    if (midPoint >= 60) {
        mid_under_flag = 1;
    }
		for (i = 81; i > 10; i -= 2) {
        under = left_edge_line[i];
        top = left_edge_line[i - 2];
				
        if (top - under >= 15 && left_edge_line[110] >= 10 && left_edge_line[i - 4] > 8 &&left_edge_line[i-6] > 8) {
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

        if (top- under >= 20) {
            leftTopPoint = i - 2;
            break;
        }
    }

    if (leftTopPoint == 0) {
        return;
    }

    draw_line(leftTopPoint, SEARCH_IMAGE_H - 1, 1);

    if (leftTopPoint >= controlReferenceLine) {
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
        draw_line(0, SEARCH_IMAGE_H - 1, 1);
    }

    if (cnt_over >= IN_TO_OUT_MIN_FRAMES &&
        right_edge_line[50] > 180 &&
        right_edge_line[60] > 180 &&
        right_edge_line[70] > 180 &&
        right_edge_line[80] > 180 &&
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
    draw_line(0, SEARCH_IMAGE_H - 1, 1);

    if ((left_edge_line[30] > 16 ||
         left_edge_line[40] > 16 ||
         left_edge_line[50] > 16) &&
        right_edge_line[40] < 170 &&
        right_edge_line[50] < 170 &&
        right_edge_line[60] < 170) {
        Straighten_flag = 1;
    }
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

    if (isContinueLine(left_edge_line) && isContinueLine(right_edge_line)) {
        ring_over_flag = 1;
    }
} 

/* 圆环结束或异常复位时，清空所有圆环状态标志。 */
void Ring_Over(void)
{
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
    in_to_out_cnt = 0;
    current_step = 0;
    cnt_over = 0;
}
