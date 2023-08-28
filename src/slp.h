#ifndef __LSP_H__
#define __LSP_H__

#include "esp_err.h"

typedef struct
{
    int receive_windows;
    int retries;
    int device_id;
    uint8_t ack_word;
} slp_config_t;

esp_err_t slp_init(slp_config_t config);
esp_err_t slp_send_bool(const char *key, bool value);
#endif
