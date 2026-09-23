#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"

class IMUDMADriver {
public:
      struct Config {
            spi_host_device_t host; //Host 3 by default
            int pin_miso;
            int pin_mosi;
            int pin_sclk;
            int pin_cs;
            int pin_int1;
            uint32_t spi_clock_hz; //Default 1 MHz
            uint8_t burst_start_reg;
            size_t burst_data_len;
            UBaseType_t task_priority;
            BaseType_t core_id;
      };

      struct ImuRawFrame {
            int16_t accel_x, accel_y, accel_z;
            int16_t temp;
            int16_t gyro_x, gyro_y, gyro_z;
            uint32_t timestamp_us;
      };

      ImuDmaDriver(const Config &cfg);
      ~ImuDmaDriver();

      esp_err_t begin();

      // Basic synchronous register helpers for sensor configuration
      esp_err_t writeRegister(uint8_t reg, uint8_t val);
      esp_err_t readRegister(uint8_t reg, uint8_t *val);

protected:
      // Virtual hook: override this in a derived class for custom math / AHRS / EKF
      virtual void onDataReady(const ImuRawFrame &frame);

private:
      Config _cfg;
      size_t _total_dma_len;

      spi_device_handle_t _spi_handle;
      spi_transaction_t       _dma_trans;
      uint8_t                                    *_tx_dma_buf;
      uint8_t                                    *_rx_dma_buf;

      TaskHandle_t                        _hTask;

      // Static callbacks and trampolines
      static void IRAM_ATTR gpio_isr_handler(void *arg);
      static void IRAM_ATTR spi_post_cb(spi_transaction_t *trans);
      static void task_entry(void *pvParameters);
};