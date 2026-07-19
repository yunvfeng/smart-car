#ifndef __TOF_AVOIDANCE_H_
#define __TOF_AVOIDANCE_H_

#include "zf_common_headfile.h"

/* Poll once every three processed camera frames. */
#define TOF_AVOID_POLL_FRAMES              3u
#define TOF_AVOID_COMM_ERROR_LIMIT         3u
#define TOF_AVOID_NO_DATA_LIMIT            12u

/* Consecutive valid samples confirm both state transitions. */
#define TOF_AVOID_TRIGGER_MIN_MM           470u
#define TOF_AVOID_TRIGGER_MAX_MM           530u
#define TOF_AVOID_ENTER_SAMPLES            2u
#define TOF_AVOID_EXIT_SAMPLES             15u

/* Reuse one existing lane row; no extra image scan is performed. */
#define TOF_AVOID_VISION_ROW               90u
#define TOF_AVOID_VISION_CENTER_DELTA_PX   6u

#define TOF_AVOID_PATH_BIAS_PX             40
#define TOF_AVOID_SPEED_CAP                165

typedef enum
{
    TOF_AVOID_STATE_FOLLOW = 0,
    TOF_AVOID_STATE_AVOID
} tof_avoid_state_enum;

/* Returns 0 on success and 1 on failure. */
uint8 TofAvoid_Init(void);
/* Update the left/right choice from the latest completed camera frame. */
void TofAvoid_OnFrame(void);
/* Low-rate DL1B polling; call after processing a camera frame. */
void TofAvoid_ServiceMain(void);
/* Ring transitions are paused only while bypassing an obstacle. */
uint8 TofAvoid_InhibitRing(void);

void TofAvoid_GetDebug(uint8 *valid, uint16 *distance_mm, uint8 *state);

#endif
