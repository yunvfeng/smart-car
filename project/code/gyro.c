#include "gyro.h"
#include "zf_device_imu660rb.h"

static volatile uint8 gyro_ready = 0;
static volatile int16 gyro_servo_correction = 0;
static int16 gyro_z_bias = 0;
static int32 gyro_z_filtered = 0;

volatile int16 g_gyro_z_for_servo = 0;

static int32 divide_by_four_toward_zero(int32 value)
{
    if (value >= 0) {
        return value >> 2;
    }
    return -(int32)(((uint32)(-value)) >> 2);
}

uint8 Gyro_Init(void)
{
    uint16 i;
    int32 sample_sum;

    gyro_ready = 0;
    gyro_servo_correction = 0;
    gyro_z_bias = 0;
    gyro_z_filtered = 0;
    g_gyro_z_for_servo = 0;

    if (imu660rb_init()) {
        return 1;
    }

    sample_sum = 0;
    for (i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        imu660rb_get_gyro();
        sample_sum += imu660rb_gyro_z;
        system_delay_ms(GYRO_CALIBRATION_DELAY_MS);
    }

    gyro_z_bias = (int16)(sample_sum / (int32)GYRO_CALIBRATION_SAMPLES);
    gyro_ready = 1;
    return 0;
}

void Gyro_Update(void)
{
    int32 corrected;

    if (!gyro_ready) {
        g_gyro_z_for_servo = 0;
        gyro_servo_correction = 0;
        return;
    }

    imu660rb_get_gyro();
    corrected = (int32)imu660rb_gyro_z - (int32)gyro_z_bias;
    if (corrected > 32767L) {
        corrected = 32767L;
    } else if (corrected < -32768L) {
        corrected = -32768L;
    }

    gyro_z_filtered = divide_by_four_toward_zero(gyro_z_filtered * 3L + corrected);
    g_gyro_z_for_servo = (int16)gyro_z_filtered;
}

void Gyro_Set_Servo_Correction(int16 correction)
{
    gyro_servo_correction = gyro_ready ? correction : 0;
}

void Gyro_Get_Debug(uint8 *ready, int16 *filtered_z, int16 *correction)
{
    bit interrupt_state;

    interrupt_state = EA;
    EA = 0;
    if (ready != NULL) {
        *ready = gyro_ready;
    }
    if (filtered_z != NULL) {
        *filtered_z = g_gyro_z_for_servo;
    }
    if (correction != NULL) {
        *correction = gyro_servo_correction;
    }
    EA = interrupt_state;
}
