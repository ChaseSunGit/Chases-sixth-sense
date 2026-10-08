/**
 * \file      SixthSense_SPI.h
 * \brief     Library for generic SPI functions for setting up devices and DMA
 *
 * \authors   Chase Sun
 */

#ifndef SIXTHSENSE_SPI_H
#define SIXTHSENSE_SPI_H

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"

bool SPI_Bus_Init(int mosi_pin, 
                  int miso_pin, 
                  int sclk_pin, 
                  int max_transfer_sz = 4096,
                  spi_host_device_t host = SPI2_HOST);

bool SPI_Add_Device(int cs_pin,
                    spi_device_handle_t &out_handle,
                    int clock_speed_hz = 10000000,
                    int mode = 0,
                    int queue_size = 3,
                    spi_host_device_t host = SPI2_HOST);

#endif // SIXTHSENSE_SPI_H