#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_check.h"

#include "vl53l0x_api.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define VL53L0X_I2C_ADDRESS_DEFAULT 0x29

static const char TAG[] = "laser";

static VL53L0X_Dev_t dev;

static void laser_reset()
{
    ESP_ERROR_CHECK(gpio_set_level(LASER_XSHUT, 0));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(gpio_set_level(LASER_XSHUT, 1));
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void laser_i2c_init()
{
    dev.i2c_port_num = LASER_I2C_PORT;
    dev.i2c_address = VL53L0X_I2C_ADDRESS_DEFAULT;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C0_PIN_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C0_PIN_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 200000,
        },
    };
    ESP_ERROR_CHECK(i2c_param_config(LASER_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(LASER_I2C_PORT, conf.mode, 0, 0, 0));
}

static VL53L0X_Error laser_calibrate(VL53L0X_Dev_t *device)
{
    VL53L0X_Error status;
    uint8_t is_aperture_spads, phase_cal, vhv_settings;
    uint32_t ref_spad_count;

    // Device Initialization (~40ms)
    status = VL53L0X_DataInit(device);
    if (status != VL53L0X_ERROR_NONE)
        return status;
    status = VL53L0X_StaticInit(device);
    if (status != VL53L0X_ERROR_NONE)
        return status;
    // SPADs calibration (~10ms)
    status = VL53L0X_PerformRefSpadManagement(device, &ref_spad_count, &is_aperture_spads);
    ESP_LOGI(TAG, "ref_spad_count = %d", ref_spad_count);
    ESP_LOGI(TAG, "is_aperture_spads = %d", is_aperture_spads);
    if (status != VL53L0X_ERROR_NONE)
        return status;
    // Temperature calibration (~40ms)
    status = VL53L0X_PerformRefCalibration(device, &vhv_settings, &phase_cal);
    if (status != VL53L0X_ERROR_NONE)
        return status;
    // Setup in single ranging mode
    status = VL53L0X_SetDeviceMode(device, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
        return status;
    return status;
}

static void laser_pins_init()
{
    ESP_ERROR_CHECK(gpio_set_direction(LASER_XSHUT, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(LASER_XSHUT, 1));
    ESP_ERROR_CHECK(gpio_set_direction(LASER_INT, GPIO_MODE_INPUT));
}

void laser_shutdown()
{
    ESP_ERROR_CHECK(gpio_set_direction(LASER_XSHUT, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(LASER_XSHUT, 0));
}

void laser_init()
{
    VL53L0X_Error err;
    laser_pins_init();
    laser_i2c_init();
    laser_reset();
    err = laser_calibrate(&dev);
    if (err != VL53L0X_ERROR_NONE)
    {
        ESP_LOGE(TAG, "laser_calibrate %d", err);
        abort();
    }
    err = VL53L0X_SetGpioConfig(
        &dev, 0,
        VL53L0X_DEVICEMODE_SINGLE_RANGING,
        VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
        VL53L0X_INTERRUPTPOLARITY_HIGH);
    if (err != VL53L0X_ERROR_NONE)
    {
        ESP_LOGE(TAG, "VL53L0X_SetGpioConfig %d", err);
        abort();
    }
}

esp_err_t laser_read_distance(uint16_t *distance)
{
    VL53L0X_Error status;
    VL53L0X_RangingMeasurementData_t measurement_data;
    gpio_wakeup_enable(LASER_INT, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    status = VL53L0X_SetDeviceMode(&dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
        return ESP_FAIL;
    status = VL53L0X_StartMeasurement(&dev);
    if (status != VL53L0X_ERROR_NONE)
        return ESP_FAIL;

    esp_light_sleep_start();
    status = VL53L0X_GetRangingMeasurementData(&dev, &measurement_data);
    VL53L0X_ClearInterruptMask(&dev, 0);
    if (status != VL53L0X_ERROR_NONE)
        return ESP_FAIL;
    if (measurement_data.RangeStatus != 0)
        return ESP_FAIL;

    *distance = measurement_data.RangeMilliMeter;
    return ESP_OK;
}

bool laser_detect_nearby_object(uint16_t threshold)
{
    uint16_t distance;
    if (laser_read_distance(&distance) != ESP_OK)
        return false;
    if (distance > threshold)
        return false;
    return true;
}
