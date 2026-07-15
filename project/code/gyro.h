#ifndef __GYRO_H_
#define __GYRO_H_

#include "zf_common_headfile.h"

#define GYRO_CALIBRATION_SAMPLES          100u
#define GYRO_CALIBRATION_DELAY_MS         5u
#define GYRO_SERVO_CORRECTION_LIMIT       40

/* 初始化 IMU660RB 并在静止状态下校准 Z 轴零偏：0 成功，非 0 失败。 */
uint8 Gyro_Init(void);

/* 采样 Z 轴并更新给舵机的低通滤波值。 */
void Gyro_Update(void);

/* PID 写入本周期实际使用的限幅横摆修正量。 */
void Gyro_Set_Servo_Correction(int16 correction);

/* 原子读取 IMU 状态、滤波 Z 轴和实际舵机修正量。 */
void Gyro_Get_Debug(uint8 *ready, int16 *filtered_z, int16 *correction);

extern volatile int16 g_gyro_z_for_servo;

#endif
