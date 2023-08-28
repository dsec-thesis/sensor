#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"

#include "qmc5883l.h"
#include "qmc.h"

#define CONFIG_MOVING_AVERAGE_LEN 10

typedef struct
{
    int measures[CONFIG_MOVING_AVERAGE_LEN];
    int sum;
    int index;
    int value;
} qmc_moving_average_t;

static const char TAG[] = "qmc";
static qmc5883l_t dev;
static qmc_moving_average_t ma;

static void qmc_moving_average_init()
{
    memset(ma.measures, 0, CONFIG_MOVING_AVERAGE_LEN * sizeof(int16_t));
    ma.sum = 0;
    ma.index = 0;
    ma.value = 0;
}

static void qmc_update_moving_average(int raw_measure)
{
    ma.sum = ma.sum - ma.measures[ma.index];
    ma.measures[ma.index] = raw_measure;
    ma.sum = ma.sum + raw_measure;
    ma.index = (ma.index + 1) % CONFIG_MOVING_AVERAGE_LEN;
    ma.value = ma.sum / CONFIG_MOVING_AVERAGE_LEN;
}

static int qmc_get_moving_average_value()
{
    return ma.value;
}

esp_err_t qmc_detect_anomaly(bool *detected, int bw)
{
    qmc5883l_raw_data_t raw_data;
    bool ready = false;
    ESP_ERROR_CHECK(qmc5883l_set_mode(&dev, QMC5883L_MODE_CONTINUOUS));
    while (!ready)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        qmc5883l_data_ready(&dev, &ready);
    }
    ESP_ERROR_CHECK(qmc5883l_get_raw_data(&dev, &raw_data));
    *detected = abs(qmc_get_moving_average_value() - raw_data.z) >= bw;
    qmc_update_moving_average(raw_data.z);
    return ESP_OK;
}

esp_err_t qmc_init()
{
    memset(&dev, 0, sizeof(qmc5883l_t));
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(qmc5883l_init_desc(&dev, QMC5883L_I2C_ADDR_DEF, 0, I2C0_PIN_SDA, I2C0_PIN_SCL));
    dev.i2c_dev.cfg.master.clk_speed = 200000; // default frequency is too high

    while (qmc5883l_set_config(&dev, QMC5883L_DR_10, QMC5883L_OSR_64, QMC5883L_RNG_2))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    qmc_moving_average_init();

    bool detected = true;
    while (detected)
    {
        qmc_detect_anomaly(&detected, 10);
    }
    ESP_LOGI(TAG, "qmc init finished");
    return ESP_OK;
}
