#include "key.h"

uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

uint8 key1_last_status = 1;
uint8 key2_last_status = 1;
uint8 key3_last_status = 1;
uint8 key4_last_status = 1;

uint8 key1_flag = 0;
uint8 key2_flag = 0;
uint8 key3_flag = 0;
uint8 key4_flag = 0;

/* 初始化四个按键和两个拨码开关，全部使用上拉输入。 */
void Key_Init(void)
{
    gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, 1, GPI_PULL_UP);

    gpio_init(SWITCH1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(SWITCH2_PIN, GPI, 1, GPI_PULL_UP);
}

/* 读取按键电平，并在按键松开沿产生一次标志位。 */
void Key_GetNum(void)
{
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;

    key1_status = gpio_get_level(KEY1_PIN);
    key2_status = gpio_get_level(KEY2_PIN);
    key3_status = gpio_get_level(KEY3_PIN);
    key4_status = gpio_get_level(KEY4_PIN);

    if (key1_status && !key1_last_status) key1_flag = 1;
    if (key2_status && !key2_last_status) key2_flag = 1;
    if (key3_status && !key3_last_status) key3_flag = 1;
    if (key4_status && !key4_last_status) key4_flag = 1;
}

/* 处理按键标志位；当前只清除标志，功能可以在这里继续扩展。 */
void Key_Anlysis(void)
{
    Key_GetNum();

    if (key1_flag) {
        key1_flag = 0;
    }

    if (key2_flag) {
        key2_flag = 0;
    }

    if (key3_flag) {
        key3_flag = 0;
    }

    if (key4_flag) {
        key4_flag = 0;
    }
}
