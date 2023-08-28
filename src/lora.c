#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "lora.h"

// Register definitions
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_VERSION 0x42

// Transceiver modes
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

// PA configuration
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40

#define REG_INVERT_IQ 0x33

#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

#define TIMEOUT_RESET 100

// SPI Stuff
#if CONFIG_SPI2_HOST
#define HOST_ID SPI2_HOST
#elif LORA_SPI3_HOST
#define HOST_ID SPI3_HOST
#endif

#define TAG "LORA"

static spi_device_handle_t spi_device_handle;
static lora_config_t lora_config;

/*
 * Configure CPU hardware to communicate with the radio chip
 */
static esp_err_t lora_pins_init()
{
   ESP_RETURN_ON_ERROR(gpio_reset_pin(LORA_RST_GPIO), TAG, "reset pin rst");
   ESP_RETURN_ON_ERROR(gpio_set_direction(LORA_RST_GPIO, GPIO_MODE_OUTPUT), TAG, "set rst as output");
   ESP_RETURN_ON_ERROR(gpio_reset_pin(LORA_CS_GPIO), TAG, "reset pin cs");
   ESP_RETURN_ON_ERROR(gpio_set_direction(LORA_CS_GPIO, GPIO_MODE_OUTPUT), TAG, "set cs as output");
   ESP_RETURN_ON_ERROR(gpio_set_level(LORA_CS_GPIO, 1), TAG, "set cs to 1");
   return ESP_OK;
}

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
static esp_err_t lora_write_reg(uint8_t reg, uint8_t val)
{
   spi_transaction_t spi_transaction = {
       .flags = 0,
       .cmd = 1,
       .addr = reg,
       .length = 8,
       .tx_buffer = &val,
       .rx_buffer = NULL,
   };
   return spi_device_transmit(spi_device_handle, &spi_transaction);
}

/**
 * Write a buffer to a register.
 * @param reg Register index.
 * @param val Value to write.
 * @param len Byte length to write.
 */
static esp_err_t lora_write_reg_buffer(uint8_t reg, uint8_t *val, uint8_t len)
{
   spi_transaction_t spi_transaction = {
       .flags = 0,
       .cmd = 1,
       .addr = reg,
       .length = 8 * len,
       .tx_buffer = val,
       .rx_buffer = NULL,
   };
   return spi_device_transmit(spi_device_handle, &spi_transaction);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @param buff buffer where to write the register value.
 * @return Value of the register.
 */
static esp_err_t lora_read_reg(uint8_t reg, uint8_t *buff)
{
   spi_transaction_t spi_transaction = {
       .flags = 0,
       .cmd = 0,
       .addr = reg,
       .length = 8,
       .tx_buffer = NULL,
       .rx_buffer = buff,
   };
   return spi_device_transmit(spi_device_handle, &spi_transaction);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @param len Byte length to read.
 * @return Value of the register.
 */
static esp_err_t lora_read_reg_buffer(uint8_t reg, uint8_t *val, uint8_t len)
{
   spi_transaction_t spi_transaction = {
       .flags = 0,
       .cmd = 0,
       .addr = reg,
       .length = 8 * len,
       .tx_buffer = NULL,
       .rx_buffer = val,
   };
   return spi_device_transmit(spi_device_handle, &spi_transaction);
}

static void lora_apply_base_config(lora_base_config_t config)
{
   lora_set_frequency(config.frequency);
   lora_set_spreading_factor(config.spreading_factor);
   lora_set_bandwidth(config.bandwidth);
   lora_set_coding_rate(config.coding_rate);
   lora_set_preamble_length(config.preamble_length);
   lora_set_sync_word(config.sync_word);
}

static void lora_apply_rx_config(lora_rx_config_t rx_config)
{
   lora_apply_base_config(rx_config.base);
}

static void lora_apply_tx_config(lora_tx_config_t tx_config)
{
   lora_apply_base_config(tx_config.base);
}

int32_t lora_receive(uint8_t *buffer, uint8_t length, TickType_t ticks_to_wait)
{
   uint8_t irq_flags;
   uint8_t fifo_rx_current_addr;
   uint8_t received_length = 0;
   TimeOut_t time_out;
   vTaskSetTimeOutState(&time_out);
   lora_apply_rx_config(lora_config.rx);
   lora_start_receive();

   while (1)
   {
      lora_read_reg(REG_IRQ_FLAGS, &irq_flags);
      if ((irq_flags & IRQ_RX_DONE_MASK) && !(irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK))
         break;
      if (xTaskCheckForTimeOut(&time_out, &ticks_to_wait))
         return -1;
      vTaskDelay(1);
   }
   lora_read_reg(REG_RX_NB_BYTES, &received_length);
   lora_idle();
   lora_read_reg(REG_FIFO_RX_CURRENT_ADDR, &fifo_rx_current_addr);
   lora_write_reg(REG_FIFO_ADDR_PTR, fifo_rx_current_addr);
   lora_read_reg_buffer(REG_FIFO, buffer, length);
   return (int32_t)received_length;
}

/**
 * Perform physical reset on the Lora chip
 */
void lora_reset(void)
{
   gpio_set_level(LORA_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(LORA_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void lora_explicit_header_mode(void)
{
   uint8_t modem_config_1;
   lora_read_reg(REG_MODEM_CONFIG_1, &modem_config_1);
   lora_write_reg(REG_MODEM_CONFIG_1, modem_config_1 & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void lora_implicit_header_mode(int32_t size)
{
   uint8_t modem_config_1;
   lora_read_reg(REG_MODEM_CONFIG_1, &modem_config_1);
   lora_write_reg(REG_MODEM_CONFIG_1, modem_config_1 | 0x01);
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void lora_idle(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void lora_sleep(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void lora_start_receive(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void lora_set_tx_power(int32_t level)
{
   // RF9x module uses PA_BOOST pin
   if (level < 2)
   {
      level = 2;
   }
   else if (level > 17)
   {
      level = 17;
   }
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void lora_set_frequency(long frequency)
{
   // rx_config.frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void lora_set_spreading_factor(int32_t sf)
{
   if (sf < 6)
   {
      sf = 6;
   }
   else if (sf > 12)
   {
      sf = 12;
   }

   if (sf == 6)
   {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   }
   else
   {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }
   uint8_t modem_config_2;
   lora_read_reg(REG_MODEM_CONFIG_2, &modem_config_2);
   lora_write_reg(REG_MODEM_CONFIG_2, (modem_config_2 & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Get spreading factor.
 */
int32_t lora_get_spreading_factor(void)
{
   uint8_t modem_config_2;
   lora_read_reg(REG_MODEM_CONFIG_2, &modem_config_2);
   return (modem_config_2 >> 4);
}

/**
 * Set Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 * @param mode mode of DIO(0 to 3)
 */
void lora_set_dio_mapping(int32_t dio, int32_t mode)
{
   uint8_t _mode;
   if (dio < 4)
   {
      lora_read_reg(REG_DIO_MAPPING_1, &_mode);
      if (dio == 0)
      {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      }
      else if (dio == 1)
      {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      else if (dio == 2)
      {
         _mode = _mode & 0xF3;
         _mode = _mode | (mode << 2);
      }
      else if (dio == 3)
      {
         _mode = _mode & 0xFC;
         _mode = _mode | mode;
      }
      lora_write_reg(REG_DIO_MAPPING_1, _mode);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
   }
   else if (dio < 6)
   {
      lora_read_reg(REG_DIO_MAPPING_2, &_mode);
      if (dio == 4)
      {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      }
      else if (dio == 5)
      {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      lora_write_reg(REG_DIO_MAPPING_2, _mode);
   }
}

/**
 * Get Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 */
int32_t lora_get_dio_mapping(int32_t dio)
{
   uint8_t _mode;
   if (dio < 4)
   {
      lora_read_reg(REG_DIO_MAPPING_1, &_mode);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
      if (dio == 0)
      {
         return ((_mode >> 6) & 0x03);
      }
      else if (dio == 1)
      {
         return ((_mode >> 4) & 0x03);
      }
      else if (dio == 2)
      {
         return ((_mode >> 2) & 0x03);
      }
      else if (dio == 3)
      {
         return (_mode & 0x03);
      }
   }
   else if (dio < 6)
   {
      lora_read_reg(REG_DIO_MAPPING_2, &_mode);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      if (dio == 4)
      {
         return ((_mode >> 6) & 0x03);
      }
      else if (dio == 5)
      {
         return ((_mode >> 4) & 0x03);
      }
   }
   return 0;
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
void lora_set_bandwidth(int32_t sbw)
{
   if (sbw < 10)
   {
      uint8_t modem_config_1;
      lora_read_reg(REG_MODEM_CONFIG_1, &modem_config_1);
      lora_write_reg(REG_MODEM_CONFIG_1, (modem_config_1 & 0x0f) | (sbw << 4));
   }
}

/**
 * Get bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
int32_t lora_get_bandwidth(void)
{
   uint8_t modem_config_1;
   lora_read_reg(REG_MODEM_CONFIG_1, &modem_config_1);
   return ((modem_config_1 & 0xf0) >> 4);
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
void lora_set_coding_rate(int32_t denominator)
{
   if (denominator < 5)
   {
      denominator = 5;
   }
   else if (denominator > 8)
   {
      denominator = 8;
   }

   int32_t cr = denominator - 4;
   uint8_t modem_config_1;
   lora_read_reg(REG_MODEM_CONFIG_1, &modem_config_1);
   lora_write_reg(REG_MODEM_CONFIG_1, (modem_config_1 & 0xf1) | (cr << 1));
}

/**
 * Get coding rate
 */
int32_t lora_get_coding_rate(void)
{
   uint8_t modem_config_1;
   lora_read_reg(REG_MODEM_CONFIG_1, &modem_config_1);
   return ((modem_config_1 & 0x0E) >> 1);
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void lora_set_preamble_length(uint32_t length)
{
   lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Get the size of preamble.
 */
uint16_t lora_get_preamble_length(void)
{
   uint8_t preamble_msb, preamble_lsb;
   lora_read_reg(REG_PREAMBLE_MSB, &preamble_msb);
   lora_read_reg(REG_PREAMBLE_LSB, &preamble_lsb);
   return (preamble_msb << 8) + preamble_lsb;
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void lora_set_sync_word(int32_t sw)
{
   lora_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void lora_enable_crc(void)
{
   uint8_t modem_config_1;
   lora_read_reg(REG_MODEM_CONFIG_2, &modem_config_1);
   lora_write_reg(REG_MODEM_CONFIG_2, modem_config_1 | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void lora_disable_crc(void)
{
   uint8_t modem_config_2;
   lora_read_reg(REG_MODEM_CONFIG_2, &modem_config_2);
   lora_write_reg(REG_MODEM_CONFIG_2, modem_config_2 & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int32_t lora_init(lora_config_t config)
{
   esp_err_t ret;
   lora_config = config;
   ESP_RETURN_ON_ERROR(lora_pins_init(), TAG, "pins init");
   spi_bus_config_t bus = {
       .miso_io_num = LORA_MISO_GPIO,
       .mosi_io_num = LORA_MOSI_GPIO,
       .sclk_io_num = LORA_SCK_GPIO,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0};
   ret = spi_bus_initialize(HOST_ID, &bus, SPI_DMA_CH_AUTO);
   assert(ret == ESP_OK);
   spi_device_interface_config_t dev = {
       .command_bits = 1,
       .address_bits = 7,
       .clock_speed_hz = 9000000,
       .mode = 0,
       .spics_io_num = LORA_CS_GPIO,
       .queue_size = 7,
       .flags = 0,
       .pre_cb = NULL};
   ret = spi_bus_add_device(HOST_ID, &dev, &spi_device_handle);
   assert(ret == ESP_OK);

   lora_reset();

   // Check version.
   uint8_t version;
   uint8_t i = 0;
   while (i++ < TIMEOUT_RESET)
   {
      lora_read_reg(REG_VERSION, &version);
      ESP_LOGD(TAG, "version=0x%02x", version);
      if (version == 0x12)
      {
         break;
      }
      vTaskDelay(2);
   }
   ESP_LOGD(TAG, "i=%d, TIMEOUT_RESET=%d", i, TIMEOUT_RESET);
   if (i == TIMEOUT_RESET + 1)
      return 0; // Illegal version

   // Default configuration.
   lora_sleep();
   lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);

   uint8_t lna;
   lora_read_reg(REG_LNA, &lna);
   lora_write_reg(REG_LNA, lna | (0x20 | 0x03));
   lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
   lora_set_tx_power(17);

   uint8_t reg_invert_iq;
   lora_read_reg(REG_INVERT_IQ, &reg_invert_iq);
   lora_write_reg(REG_INVERT_IQ, reg_invert_iq | 0x01);

   lora_explicit_header_mode();
   lora_idle();
   return 1;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int32_t lora_received(void)
{
   uint8_t irq_flags;
   lora_read_reg(REG_IRQ_FLAGS, &irq_flags);
   return (int32_t)irq_flags & IRQ_RX_DONE_MASK;
}

/**
 * Returns RegIrqFlags.
 */
int32_t lora_get_irq(void)
{
   uint8_t irq_flags;
   lora_read_reg(REG_IRQ_FLAGS, &irq_flags);
   return irq_flags;
}

/**
 * Return last packet's RSSI.
 */
int32_t lora_packet_rssi(void)
{
   uint8_t pkg_rssi;
   lora_read_reg(REG_PKT_RSSI_VALUE, &pkg_rssi);
   return (pkg_rssi - (lora_config.rx.base.frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float lora_packet_snr(void)
{
   uint8_t pkg_snr;
   lora_read_reg(REG_PKT_SNR_VALUE, &pkg_snr);
   return pkg_snr * 0.25;
}

/**
 * Shutdown hardware.
 */
void lora_close(void)
{
   lora_sleep();
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
BaseType_t lora_send(uint8_t *buffer, int32_t length)
{
   // Transfer data to radio.
   lora_idle();
   lora_write_reg(REG_FIFO_ADDR_PTR, 0);
   lora_write_reg_buffer(REG_FIFO, buffer, length);
   lora_write_reg(REG_PAYLOAD_LENGTH, length);

   lora_apply_tx_config(lora_config.tx);

   // Start transmission and wait for conclusion.
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   uint8_t irq_flags = 0;
   lora_read_reg(REG_IRQ_FLAGS, &irq_flags);
   while ((irq_flags & IRQ_TX_DONE_MASK) == 0)
   {
      portYIELD();
      lora_read_reg(REG_IRQ_FLAGS, &irq_flags);
   }
   lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

   return pdTRUE;
}
