#include "assistant_debug.h"

#if ASSISTANT_DEBUG_ENABLE

#include "image.h"
#include "gyro.h"
#include "laser.h"
#include "motor.h"
#include "pid.h"
#include "ring.h"

#define ASSISTANT_PARAM_MIN_CHANNEL     1u
#define ASSISTANT_PARAM_MAX_CHANNEL     SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT
#define ASSISTANT_EXPOSURE_MIN          1.0f
#define ASSISTANT_EXPOSURE_MAX          4095.0f
#define ASSISTANT_SPEED_MIN              165.0f
#define ASSISTANT_GYRO_KG_MIN            (-32.0f)
#define ASSISTANT_GYRO_KG_MAX            32.0f

static uint8 assistant_image_cnt = 0;
static uint8 assistant_scope_cnt = 0;
static volatile uint16 assistant_time_100us = 0;
static volatile uint16 assistant_image_process_time_100us = 0;

#if ASSISTANT_DEBUG_IMAGE_ENABLE && ASSISTANT_DEBUG_IMAGE_OVERLAY_ENABLE
static const char *Assistant_Ring_State_Name(void)
{
    switch (current_step) {
    case 0: return "NORM";
    case 1: return "FIRST";
    case 2: return "ENTER";
    case 3: return "TURN";
    case 4: return "IN";
    case 5: return "OUT";
    case 6: return "BACK";
    case 7: return "OVER";
    default: return "ERR";
    }
}

static void Assistant_Draw_Image_Point(uint8 x, uint8 y, uint8 gray)
{
    if (x < MT9V03X_W && y < MT9V03X_H) {
        mt9v03x_image[y][x] = gray;
    }
}

static void Assistant_Draw_Image_Char(uint8 x, uint8 y, char dat)
{
    uint8 row;
    uint8 col;
    uint8 temp;
    uint8 far *p;

    if (dat < ' ' || dat > '~') {
        dat = ' ';
    }
    if (x + 8u > MT9V03X_W || y + 16u > MT9V03X_H) {
        return;
    }

    for (row = 0; row < 16; row++) {
        temp = ascii_font_8x16[dat - 32][row];
        p = &mt9v03x_image[y + row][x];
        for (col = 0; col < 8; col++) {
            *p++ = (temp & 0x01) ? 255 : 0;
            temp >>= 1;
        }
    }
}

static uint8 Assistant_Draw_Image_String(uint8 x, uint8 y, const char *str)
{
    while (*str && x + 8u <= MT9V03X_W) {
        Assistant_Draw_Image_Char(x, y, *str++);
        x += 8;
    }

    return x;
}

static void Assistant_Draw_Target_Cross(void)
{
    uint8 i;

    if (!tar_flag) {
        return;
    }

    for (i = 0; i <= 4; i++) {
        Assistant_Draw_Image_Point((uint8)(tar_x - 2 + i), tar_y, 255);
        Assistant_Draw_Image_Point(tar_x, (uint8)(tar_y - 2 + i), 255);
    }
}

static void Assistant_Draw_Status_Overlay(void)
{
    uint8 x;

    x = 2;
    x = Assistant_Draw_Image_String(x, 2, "R:");
    Assistant_Draw_Image_String(x, 2, Assistant_Ring_State_Name());

    Assistant_Draw_Target_Cross();
}
#endif

static void Assistant_Send_Camera(void)
{
#if ASSISTANT_DEBUG_IMAGE_ENABLE && ASSISTANT_DEBUG_IMAGE_OVERLAY_ENABLE
    Assistant_Draw_Status_Overlay();
#endif
    seekfree_assistant_camera_send();
}

static float Assistant_Clamp_Float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int32 Assistant_Float_To_Int32(float value)
{
    return (int32)value;
}

static uint8 Assistant_Frame_Div_Hit(uint8 *counter, uint8 divider)
{
    if (divider <= 1u) {
        return 1;
    }

    (*counter)++;
    if (*counter >= divider) {
        *counter = 0;
        return 1;
    }

    return 0;
}

static void Assistant_Load_Default_Params(void)
{
    int16 speed_min;
    int16 speed_max;

    Motor_Get_Speed_Range(&speed_min, &speed_max);
    seekfree_assistant_parameter[ASSISTANT_PARAM_SERVO_KP - 1] =
        (float)servo_pidf.kp;
    seekfree_assistant_parameter[ASSISTANT_PARAM_SERVO_KD - 1] =
        (float)servo_pidf.kd;
    seekfree_assistant_parameter[ASSISTANT_PARAM_GYRO_KG - 1] =
        (float)PID_Get_Servo_Gyro_Gain();
    seekfree_assistant_parameter[ASSISTANT_PARAM_MOTOR_KP - 1] =
        (float)pid_lf.kp;
    seekfree_assistant_parameter[ASSISTANT_PARAM_MOTOR_KI - 1] =
        (float)pid_lf.ki;
    seekfree_assistant_parameter[ASSISTANT_PARAM_MIN_SPEED - 1] =
        (float)speed_min;
    seekfree_assistant_parameter[ASSISTANT_PARAM_MAX_SPEED - 1] =
        (float)speed_max;
    seekfree_assistant_parameter[ASSISTANT_PARAM_CAMERA_EXPOSURE - 1] =
        (float)camera_exposure_time;
}

static void Assistant_Apply_Param(uint8 channel, float value)
{
    int32 fixed_value;
    uint16 exposure;
    int16 speed_min;
    int16 speed_max;

    switch (channel) {
    case ASSISTANT_PARAM_SERVO_KP:
        servo_pidf.kp = Assistant_Float_To_Int32(value);
        break;

    case ASSISTANT_PARAM_SERVO_KD:
        servo_pidf.kd = Assistant_Float_To_Int32(value);
        break;

    case ASSISTANT_PARAM_GYRO_KG:
        value = Assistant_Clamp_Float(value, ASSISTANT_GYRO_KG_MIN, ASSISTANT_GYRO_KG_MAX);
        if (value >= 0.0f) {
            fixed_value = (int32)(value + 0.5f);
        } else {
            fixed_value = (int32)(value - 0.5f);
        }
        PID_Set_Servo_Gyro_Gain((int16)fixed_value);
        seekfree_assistant_parameter[ASSISTANT_PARAM_GYRO_KG - 1] =
            (float)PID_Get_Servo_Gyro_Gain();
        break;

    case ASSISTANT_PARAM_MOTOR_KP:
        fixed_value = Assistant_Float_To_Int32(value);
        pid_lf.kp = fixed_value;
        pid_rf.kp = fixed_value;
        break;

    case ASSISTANT_PARAM_MOTOR_KI:
        fixed_value = Assistant_Float_To_Int32(value);
        pid_lf.ki = fixed_value;
        pid_rf.ki = fixed_value;
        break;

    case ASSISTANT_PARAM_MIN_SPEED:
        value = Assistant_Clamp_Float(value, ASSISTANT_SPEED_MIN, (float)MAX_SPEED_TUNE_MAX);
        Motor_Get_Speed_Range(&speed_min, &speed_max);
        speed_min = (int16)(value + 0.5f);
        if (speed_max < speed_min) {
            speed_max = speed_min;
        }
        Motor_Set_Speed_Range(speed_min, speed_max);
        Motor_Get_Speed_Range(&speed_min, &speed_max);
        seekfree_assistant_parameter[ASSISTANT_PARAM_MIN_SPEED - 1] = (float)speed_min;
        seekfree_assistant_parameter[ASSISTANT_PARAM_MAX_SPEED - 1] = (float)speed_max;
        break;

    case ASSISTANT_PARAM_MAX_SPEED:
        value = Assistant_Clamp_Float(value, ASSISTANT_SPEED_MIN, (float)MAX_SPEED_TUNE_MAX);
        Motor_Get_Speed_Range(&speed_min, &speed_max);
        speed_max = (int16)(value + 0.5f);
        if (speed_min > speed_max) {
            speed_min = speed_max;
        }
        Motor_Set_Speed_Range(speed_min, speed_max);
        Motor_Get_Speed_Range(&speed_min, &speed_max);
        seekfree_assistant_parameter[ASSISTANT_PARAM_MIN_SPEED - 1] = (float)speed_min;
        seekfree_assistant_parameter[ASSISTANT_PARAM_MAX_SPEED - 1] = (float)speed_max;
        break;

    case ASSISTANT_PARAM_CAMERA_EXPOSURE:
        value = Assistant_Clamp_Float(value, ASSISTANT_EXPOSURE_MIN, ASSISTANT_EXPOSURE_MAX);
        exposure = (uint16)(value + 0.5f);
        if (exposure != camera_exposure_time) {
            camera_exposure_time = exposure;
            mt9v03x_set_exposure_time(camera_exposure_time);
        }
        break;

    default:
        break;
    }
}

static void Assistant_Apply_Param_Updates(void)
{
    uint8 i;

    for (i = ASSISTANT_PARAM_MIN_CHANNEL; i <= ASSISTANT_PARAM_MAX_CHANNEL; i++) {
        if (seekfree_assistant_parameter_update_flag[i - 1]) {
            seekfree_assistant_parameter_update_flag[i - 1] = 0;
            Assistant_Apply_Param(i, seekfree_assistant_parameter[i - 1]);
        }
    }
}

static void Assistant_Send_Scope(void)
{
    int16 mid_error;
    int16 servo_error;
    int16 gyro_z;
    int16 gyro_correction;

    mid_error = (int16)mid_line[controlReferenceLine] - (int16)Mid_Col;
    servo_error = (int16)Out_servo - (int16)SERVO_DUTY_MID;
    Gyro_Get_Debug(NULL, &gyro_z, &gyro_correction);

    seekfree_assistant_oscilloscope_data.channel_num = SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT;
    seekfree_assistant_oscilloscope_data.dat[0] = (float)mid_error;
    seekfree_assistant_oscilloscope_data.dat[1] = (float)servo_error;
    seekfree_assistant_oscilloscope_data.dat[2] = (float)encoder_data_l;
    seekfree_assistant_oscilloscope_data.dat[3] = (float)encoder_data_r;
    seekfree_assistant_oscilloscope_data.dat[4] = (float)gyro_z;
    seekfree_assistant_oscilloscope_data.dat[5] = (float)gyro_correction;
    seekfree_assistant_oscilloscope_data.dat[6] = (float)motor_pwm_l;
    seekfree_assistant_oscilloscope_data.dat[7] = (float)motor_pwm_r;

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

void Assistant_Debug_Init(void)
{
    seekfree_assistant_interface_init(ASSISTANT_DEBUG_DEVICE);
    seekfree_assistant_init();

    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                 (void *)&mt9v03x_image[0][0],
                                                 MT9V03X_W,
                                                 MT9V03X_H);
#if ASSISTANT_DEBUG_BOUNDARY_ENABLE
    seekfree_assistant_camera_boundary_config(X_BOUNDARY,
                                              SEARCH_IMAGE_H,
                                              left_control_line,
                                              mid_line,
                                              right_control_line,
                                              NULL,
                                              NULL,
                                              NULL);
#else
    seekfree_assistant_camera_boundary_config(NO_BOUNDARY,
                                              0,
                                              NULL,
                                              NULL,
                                              NULL,
                                              NULL,
                                              NULL,
                                              NULL);
#endif
    Assistant_Load_Default_Params();
}

void Assistant_Debug_Task(void)
{
#if ASSISTANT_DEBUG_PARAM_ENABLE
    seekfree_assistant_data_analysis();
    Assistant_Apply_Param_Updates();
#endif
}

void Assistant_Debug_On_Frame(void)
{
    Assistant_Debug_Task();

#if ASSISTANT_DEBUG_SCOPE_ENABLE
    if (Assistant_Frame_Div_Hit(&assistant_scope_cnt, ASSISTANT_DEBUG_SCOPE_DIV)) {
        Assistant_Send_Scope();
    }
#endif

#if ASSISTANT_DEBUG_IMAGE_ENABLE
    if (Assistant_Frame_Div_Hit(&assistant_image_cnt, ASSISTANT_DEBUG_IMAGE_DIV)) {
        Assistant_Send_Camera();
    }
#endif
}

void Assistant_Debug_Send_Now(void)
{
#if ASSISTANT_DEBUG_SCOPE_ENABLE
    Assistant_Send_Scope();
#endif
#if ASSISTANT_DEBUG_IMAGE_ENABLE
    Assistant_Send_Camera();
#endif
}

void Assistant_Debug_Timebase_Tick(void)
{
    assistant_time_100us++;
}

uint16 Assistant_Debug_Timebase_Now(void)
{
    return assistant_time_100us;
}

void Assistant_Debug_Set_Image_Process_Time(uint16 time_100us)
{
    assistant_image_process_time_100us = time_100us;
}

#else

void Assistant_Debug_Init(void)
{
}

void Assistant_Debug_Task(void)
{
}

void Assistant_Debug_On_Frame(void)
{
}

void Assistant_Debug_Send_Now(void)
{
}

void Assistant_Debug_Timebase_Tick(void)
{
}

uint16 Assistant_Debug_Timebase_Now(void)
{
    return 0;
}

void Assistant_Debug_Set_Image_Process_Time(uint16 time_100us)
{
    (void)time_100us;
}

#endif
