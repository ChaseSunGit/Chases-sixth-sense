/**
 * \file      Sixth_sense_SPI.h
 * \brief     Library for generic SPI functions for setting up devices and DMA
 *
 * \authors   Chase Sun
 */

#ifndef SIXTH_SENSE_SPI_H
#define SIXTH_SENSE_SPI_H

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"


struct SPI_DMA_Channel {
      spi_device_handle_t handle;
      uint8_t *tx_buffer;
      uint8_t *rx_buffer;
      spi_transaction_t trans;
      size_t buffer_len;
      volatile bool in_progress;
      volatile bool data_ready;
};

bool SPI_Bus_Init(int mosi_pin, 
                  int miso_pin, 
                  int sclk_pin, 
                  int max_transfer_sz = 4096,
                  spi_host_device_t host = SPI2_HOST);

bool SPI_Add_Device(int cs_pin,
                    transaction_cb_t post_callback,
                    spi_device_handle_t &out_handle,
                    int clock_speed_hz = 10000000,
                    int mode = 0,
                    int queue_size = 3,
                    spi_host_device_t host = SPI2_HOST);

bool SPI_Arm_DMA_Channel(size_t burst_len,
                         uint8_t initial_cmd_byte,
                         SPI_DMA_Channel &out_channel);

void SPI_Free_DMA_Channel(SPI_DMA_Channel &channel);

#endif // SIXTH_SENSE_SPI_H