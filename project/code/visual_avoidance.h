#ifndef __VISUAL_AVOIDANCE_H_
#define __VISUAL_AVOIDANCE_H_

#include "zf_common_headfile.h"

/* The three values normally adjusted during track testing. */
#define VISUAL_AVOID_TRIGGER_ROW       90u
#define VISUAL_AVOID_PATH_BIAS_PX      30
#define VISUAL_AVOID_PASS_PULSES       6500u

/* Fixed control values for the first low-speed version. */
#define VISUAL_AVOID_SPEED_CAP         165
#define VISUAL_AVOID_RECENTER_TICKS    10u

typedef struct
{
    uint8 valid;
    uint8 left;
    uint8 right;
    uint8 bottom_row;
} visual_avoid_candidate_t;

typedef enum
{
    VISUAL_AVOID_STATE_FOLLOW = 0,
    VISUAL_AVOID_STATE_BYPASS,
    VISUAL_AVOID_STATE_RECENTER
} visual_avoid_state_t;

void VisualAvoid_Init(void);
void VisualAvoid_OnFrame(const visual_avoid_candidate_t *candidate);
void VisualAvoid_ControlTick(int16 encoder_l, int16 encoder_r);
uint8 VisualAvoid_InhibitRing(void);
void VisualAvoid_GetDebug(uint8 *candidate_valid,
                          uint8 *left,
                          uint8 *right,
                          uint8 *bottom,
                          uint8 *state,
                          uint8 *pending_hit,
                          int8 *bias,
                          uint16 *travel);

#endif
