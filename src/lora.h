#ifndef __LORA_H__
#define __LORA_H__

#include <stdint.h>
#include "freertos/FreeRTOS.h"

typedef struct
{
    int32_t frequency;
    int32_t spreading_factor;
    int32_t bandwidth;
    int32_t coding_rate;
    int32_t preamble_length;
    bool enable_crc;
    int32_t sync_word;
} lora_base_config_t;

typedef struct
{
    uint8_t frame[256];
    int32_t length;
    int32_t time;
    int32_t rssi;
    float snr;
} lora_rx_frame_t;

typedef struct
{
    lora_base_config_t base;
    int32_t power;
} lora_rx_config_t;

typedef struct
{
    lora_base_config_t base;
    int32_t gain;
} lora_tx_config_t;

typedef struct
{
    lora_tx_config_t tx;
    lora_rx_config_t rx;
} lora_config_t;

void lora_reset(void);
void lora_explicit_header_mode(void);
void lora_implicit_header_mode(int32_t size);
void lora_idle(void);
void lora_sleep(void);
void lora_start_receive(void);
int32_t lora_get_irq(void);
void lora_set_tx_power(int32_t level);
void lora_set_frequency(long frequency);
void lora_set_spreading_factor(int32_t sf);
int32_t lora_get_spreading_factor(void);
void lora_set_dio_mapping(int32_t dio, int32_t mode);
int32_t lora_get_dio_mapping(int32_t dio);
void lora_set_bandwidth(int32_t sbw);
int32_t lora_get_bandwidth(void);
void lora_set_coding_rate(int32_t denominator);
int32_t lora_get_coding_rate(void);
void lora_set_preamble_length(uint32_t length);
uint16_t lora_get_preamble_length(void);
void lora_set_sync_word(int32_t sw);
void lora_enable_crc(void);
void lora_disable_crc(void);
int32_t lora_init(lora_config_t config);
int32_t lora_received(void);
int32_t lora_packet_rssi(void);
float lora_packet_snr(void);
void lora_close(void);

BaseType_t lora_send(uint8_t *buffer, int32_t length);
int32_t lora_receive(uint8_t *buffer, uint8_t length, TickType_t ticks_to_wait);
#endif
