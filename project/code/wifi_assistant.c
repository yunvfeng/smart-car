#include "wifi_assistant.h"

#include "key.h"
#include "zf_device_wifi_spi.h"

#define WIFI_ASSISTANT_RETRY_DELAY_MS 100
#define WIFI_ASSISTANT_RETRY_MAX      1
#define WIFI_ASSISTANT_SSID           "yunfeng11"
#define WIFI_ASSISTANT_PASS           "2b)7034A"
#define WIFI_ASSISTANT_PC_IP          "192.168.137.1"
#define WIFI_ASSISTANT_PC_PORT        "8086"
#define WIFI_ASSISTANT_LOCAL_PORT     "6666"

static uint8 wifi_assistant_ready = 0;
static uint8 wifi_assistant_tried = 0;
static wifi_assistant_status_enum wifi_assistant_status = WIFI_ASSISTANT_IDLE;

static uint8 Wifi_Assistant_Switch_On(void)
{
    return gpio_get_level(SWITCH2_PIN) ? 0 : 1;
}

static uint8 Wifi_Assistant_Connect(void)
{
    uint8 i;
    uint8 wifi_connected;

    wifi_connected = 0;

    for (i = 0; i < WIFI_ASSISTANT_RETRY_MAX; i++) {
        if (wifi_spi_init(WIFI_ASSISTANT_SSID, WIFI_ASSISTANT_PASS)) {
            continue;
        }

        wifi_connected = 1;

        if (!wifi_spi_socket_connect("TCP",
                                     WIFI_ASSISTANT_PC_IP,
                                     WIFI_ASSISTANT_PC_PORT,
                                     WIFI_ASSISTANT_LOCAL_PORT)) {
            wifi_spi_send_string("STC32 WiFi Assistant Ready!\r\n");
            return 0;
        }

        system_delay_ms(WIFI_ASSISTANT_RETRY_DELAY_MS);
    }

    return wifi_connected ? 2 : 1;
}

void Wifi_Assistant_Init(void)
{
    wifi_assistant_ready = 0;
    wifi_assistant_tried = 0;
    wifi_assistant_status = WIFI_ASSISTANT_IDLE;
}

uint8 Wifi_Assistant_Ready(void)
{
    uint8 wifi_error;

    if (!Wifi_Assistant_Switch_On()) {
        wifi_assistant_tried = 0;
        if (!wifi_assistant_ready) {
            wifi_assistant_status = WIFI_ASSISTANT_IDLE;
        }
        return 0;
    }

    if (wifi_assistant_ready) {
        return 1;
    }

    if (wifi_assistant_tried) {
        return 0;
    }

    wifi_assistant_tried = 1;
    wifi_error = Wifi_Assistant_Connect();
    if (wifi_error) {
        wifi_assistant_status = (1 == wifi_error) ?
                                WIFI_ASSISTANT_WIFI_FAIL :
                                WIFI_ASSISTANT_SOCKET_FAIL;
        return 0;
    }

    wifi_assistant_ready = 1;
    wifi_assistant_status = WIFI_ASSISTANT_READY;
    return 1;
}

wifi_assistant_status_enum Wifi_Assistant_Status(void)
{
    return wifi_assistant_status;
}
