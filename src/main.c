#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_sleep.h"

#include "lora.h"
#include "slp.h"
#include "qmc.h"
#include "laser.h"

static const char TAG[] = "main";

typedef enum
{
    LOW_POWER = 0,
    HIGH_POWER,
} power_mode_t;

typedef struct
{
    power_mode_t power_mode;
    bool taken;
} sensor_t;

static RTC_DATA_ATTR sensor_t sensor = {
    .power_mode = HIGH_POWER,
    .taken = false,
};

void low_power(void *args)
{
    bool detected = false;
    laser_shutdown();
    ESP_ERROR_CHECK(qmc_init());

    while (1)
    {
        qmc_detect_anomaly(&detected, 200);
        if (detected)
        {
            ESP_LOGI(TAG, "anomaly detected");
            sensor.power_mode = HIGH_POWER;
            esp_sleep_enable_timer_wakeup(100);
            esp_deep_sleep_start();
        }

        esp_sleep_enable_timer_wakeup(500 * 1000);
        esp_light_sleep_start();
    }
}

void high_power(void *args)
{

    slp_config_t config = {
        .receive_windows = 1000,
        .retries = 10,
        .device_id = SPACE_ID,
        .ack_word = 0x24,

    };
    slp_init(config);
    laser_init();

    bool detected = laser_detect_nearby_object(1000);
    if ((detected && !sensor.taken) || (!detected && sensor.taken))
    {
        sensor.taken = !sensor.taken;
        ESP_LOGI(TAG, "taken: %d", sensor.taken);
        slp_send_bool("taken", sensor.taken);
    }

    sensor.power_mode = LOW_POWER;
    esp_sleep_enable_timer_wakeup(100);
    esp_deep_sleep_start();
}

void app_main()
{
    switch (sensor.power_mode)
    {
    case LOW_POWER:
        low_power(NULL);
        break;
    case HIGH_POWER:
        high_power(NULL);
        break;
    }
}
