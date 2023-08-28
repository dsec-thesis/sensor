#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "cbor.h"

#include "lora.h"
#include "slp.h"

static const char *TAG = "LSP";

typedef struct
{
    int receive_windows;
    int retries;
    int device_id;
    uint8_t ack_word;
} slp_context_t;

static slp_context_t context;

esp_err_t slp_init(slp_config_t config)
{
    context.receive_windows = config.receive_windows;
    context.retries = config.retries;
    context.device_id = config.device_id;
    context.ack_word = config.ack_word;
    lora_config_t lora_config = {
        .rx = {
            .base = {
                .bandwidth = 7,
                .coding_rate = 5,
                .enable_crc = true,
                .frequency = 867900000,
                .preamble_length = 12,
                .spreading_factor = 7,
                .sync_word = 0x27,
            },
            .power = 7,
        },
        .tx = {
            .base = {
                .bandwidth = 7,
                .coding_rate = 5,
                .enable_crc = true,
                .frequency = 916800000,
                .preamble_length = 12,
                .spreading_factor = 7,
                .sync_word = 0x27,
            },
            .gain = 8,
        },
    };
    return lora_init(lora_config);
}

esp_err_t slp_send_bool(const char *key, bool value)
{
    uint8_t ack[1];
    uint8_t buffer[256];
    CborEncoder encoder, map_encoder;

    cbor_encoder_init(&encoder, buffer, sizeof(buffer), 0);
    cbor_encoder_create_map(&encoder, &map_encoder, CborIndefiniteLength);
    cbor_encode_text_stringz(&map_encoder, "id");
    cbor_encode_int(&encoder, context.device_id);
    cbor_encode_text_stringz(&map_encoder, key);
    cbor_encode_boolean(&map_encoder, value);
    cbor_encoder_close_container(&encoder, &map_encoder);
    int32_t length = cbor_encoder_get_buffer_size(&encoder, buffer);

    for (int i = 0; i < context.retries; i++)
    {
        lora_send(buffer, length);
        if (lora_receive(ack, 1, pdMS_TO_TICKS(context.receive_windows)) > 0 && ack[0] == context.ack_word)
            return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(rand() % 1000));
    }
    return ESP_FAIL;
}
