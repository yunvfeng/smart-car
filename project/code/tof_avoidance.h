#ifndef __TOF_AVOIDANCE_H_
#define __TOF_AVOIDANCE_H_

#include "zf_common_headfile.h"

/* 调试时临时改为 1；正常跑车保持 0，减少 WiFi 调试开销。 */
#define TOF_AVOID_CALIBRATION_MODE            0u

/* 固定内部值，不需要日常调节。 */
#define TOF_AVOID_POLL_TICKS                  2u
#define TOF_AVOID_TRIGGER_SAMPLES             3u
#define TOF_AVOID_INVALID_LIMIT               5u
#define TOF_AVOID_SPEED_CAP                   165
#define TOF_AVOID_CLEAR_HYSTERESIS_MM         250u

/* 极简版只需要关注这三个值。 */
#define TOF_AVOID_TRIGGER_MM                  900u
#define TOF_AVOID_RIGHT_BIAS_PX               20
#define TOF_AVOID_PASS_CLEAR_SAMPLES          20u

#define TOF_AVOID_CLEAR_MM                    (TOF_AVOID_TRIGGER_MM + TOF_AVOID_CLEAR_HYSTERESIS_MM)
#define TOF_AVOID_EMERGENCY_MM                (TOF_AVOID_TRIGGER_MM / 2u)

typedef enum
{
    TOF_AVOID_STATE_FOLLOW = 0,
    TOF_AVOID_STATE_AVOID,
    TOF_AVOID_STATE_STOPPED
} tof_avoid_state_enum;

/* 初始化 DL1B；失败时保持普通摄像头循迹。 */
uint8 TofAvoid_Init(void);
/* 主循环中调用，软件 IIC 和避障判断都只在这里执行。 */
void TofAvoid_ServiceMain(void);
/* 15 ms 中断中调用，只负责每两次请求一次 ToF 轮询。 */
void TofAvoid_ControlTick(void);
/* STOPPED 中按 KEY1，等新的安全距离样本后恢复。 */
void TofAvoid_RequestRearm(void);
/* 绕行或停车时暂停环岛、靶点识别和激光。 */
uint8 TofAvoid_InhibitFeatures(void);

void TofAvoid_GetDebug(uint8 *valid,
                       uint16 *distance_mm,
                       uint8 *state,
                       uint8 *clear_samples);

#endif
