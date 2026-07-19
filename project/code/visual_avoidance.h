#ifndef __VISUAL_AVOIDANCE_H_
#define __VISUAL_AVOIDANCE_H_

#include "zf_common_headfile.h"

/* A width change of at least 12 pixels is treated as a jump. */
#define VISUAL_AVOID_WIDTH_JUMP_PX           12
#define VISUAL_AVOID_UPPER_JUMP_PX           8
/* The two left-edge obstacle corners must differ by strictly less than 10 px. */
#define VISUAL_AVOID_LEFT_POINT_DIFF_PX      10

#define VISUAL_AVOID_RIGHT_BIAS_PX           (-40)
#define VISUAL_AVOID_PASS_PULSES             6500u
#define VISUAL_AVOID_RECENTER_TICKS          8u

#define VISUAL_AVOID_RESULT_NONE             0u
#define VISUAL_AVOID_RESULT_HOLD             1u
#define VISUAL_AVOID_RESULT_DONE             2u

void VisualAvoid_Init(void);

/*
 * Called first from Ring() only while the ring state is FIRST.
 * Returns NONE, HOLD, or DONE so Ring() owns every ring-state transition.
 */
uint8 VisualAvoid_ProcessFrame(uint8 enable);

/* Lightweight 15 ms encoder accumulation; never scans or modifies image data. */
void VisualAvoid_ControlTick(int16 encoder_l, int16 encoder_r);

void VisualAvoid_GetDebug(uint8 *detected,
                          uint8 *active,
                          uint8 *upper_row,
                          uint8 *lower_row,
                          uint8 *lower_col);

#endif
