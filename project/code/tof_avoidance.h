#ifndef __TOF_AVOIDANCE_H_
#define __TOF_AVOIDANCE_H_

#include "zf_common_headfile.h"

/* Keep software-I2C load bounded: one DL1B poll every four 15 ms ticks. */
#define TOF_AVOID_POLL_TICKS                  4u
#define TOF_AVOID_TRIGGER_SAMPLES             2u
#define TOF_AVOID_INVALID_LIMIT               5u
#define TOF_AVOID_SPEED_CAP                   165

/* First low-speed calibration values. Bias sign is selected from the camera. */
#define TOF_AVOID_TRIGGER_MIN_MM              250u
#define TOF_AVOID_TRIGGER_MM                  430u
#define TOF_AVOID_PATH_BIAS_PX                40
#define TOF_AVOID_PASS_CLEAR_SAMPLES          15u
#define TOF_AVOID_CLEAR_MM                    200u

typedef enum
{
    TOF_AVOID_STATE_FOLLOW = 0,
    TOF_AVOID_STATE_AVOID
} tof_avoid_state_enum;

typedef enum
{
    TOF_AVOID_OBSTACLE_UNKNOWN = 0,
    TOF_AVOID_OBSTACLE_LEFT,
    TOF_AVOID_OBSTACLE_RIGHT
} tof_avoid_obstacle_side_enum;

/* Returns 0 only when DL1B initialization succeeds. */
uint8 TofAvoid_Init(void);
/* Software I2C polling; call only from the main loop. */
void TofAvoid_ServiceMain(void);
/* Reuse the latest raw lane edges to classify only the obstacle side. */
void TofAvoid_OnFrame(void);
/* Lightweight scheduler and safety-cap maintenance. */
void TofAvoid_ControlTick(void);
/* Freeze ring transitions only while an avoidance maneuver is active. */
uint8 TofAvoid_InhibitRing(void);

void TofAvoid_GetDebug(uint8 *valid,
                       uint16 *distance_mm,
                       uint8 *state,
                       uint8 *clear_samples,
                       uint8 *obstacle_side);

#endif
