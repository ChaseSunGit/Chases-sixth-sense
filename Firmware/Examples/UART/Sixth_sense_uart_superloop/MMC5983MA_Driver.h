/**
 * \file MMC5983MA_Driver.h
 * \brief Header file for driving the MMC5983MA Magnetometer

 * \author Chase Sun
 */

#ifndef MMC5983MA_DRIVER_H
#define MMC5983MA_DRIVER_H

#include <Arduino.h>
#include <Math.h>
#include <String.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "Sixth_sense_SPI.h"

// CONSTANTS/MACROS
// Read length: 1 byte address + 6 bytes payload (X, Y, Z 16-bit) + 1 byte (XYZ 18-bit LSBs) = 8 bytes total, ignore temp
#define MMC_BURST_LEN   8
#define COUNTS_PER_G_18 16384.0f // 18-bit counts per Gauss for ±2G range
#define NULL_FIELD_18   131072.0f // 18-bit null field value for sensitivity compensation

// SPI Pins
#define PIN_INT_MMC     4   // Mag Data Ready Interrupt
#define PIN_MISO_MMC    8   // Sensor SDO
#define PIN_MOSI_MMC    9   // Sensor SDI
#define PIN_SCLK_MMC    10  // SCLK
#define PIN_CS_MMC      13  // CS

// MMC5983MA Registers
#define MMC_XOUT0       0x00
#define MMC_XOUT1       0x01
#define MMC_YOUT0       0x02
#define MMC_YOUT1       0x03
#define MMC_ZOUT0       0x04
#define MMC_ZOUT1       0x05
#define MMC_XYZOUT2     0x06
#define MMC_TOUT        0x07
#define MMC_STATUS      0x08
#define MMC_CTRL0       0x09
#define MMC_CTRL1       0x0A
#define MMC_CTRL2       0x0B
#define MMC_CTRL3       0x0C
#define MMC_PROD_ID     0x2F

// SPI Command Masks
#define MMC_SPI_READ_FLAG 0x80 // Bit 0 (MSB) = 1 for Read
#define MMC_SPI_ADDR_MASK 0x3F // Bits 2-7 contain the 6-bit address

// STRUCTURES
struct MMC_Data {
      // Field readings in Gauss
      float mx;
      float my;
      float mz;

      //Tracked post calibration readings
      float mx_cal;
      float my_cal;
      float mz_cal;

      //Frequency
      int reportFrequency; // The frequency at which the sensor is reporting data

      //Calibration
      //Hard iron - constant offsets
      float offset[3];
      //Soft iron - 3x3 correction Matrix
      float W[3][3];
};

// EXTERNAL GLOBAL VARIABLES
extern spi_device_handle_t spi_MMC;
extern SPI_DMA_Channel SPI_DMA_MMC;

extern volatile bool dma_in_progress_MMC;
extern volatile bool new_mag_data_ready;

extern bool MMC_first_read;

extern MMC_Data MMC_Data_Holder;

// FUNCTION PROTOTYPES
void IRAM_ATTR MMC_ISR_dataReady();
void IRAM_ATTR MMC_ISR_DMAcomplete_callback(spi_transaction_t *trans);

void MMC_write_reg(uint8_t reg, uint8_t data);
uint8_t MMC_read_reg(uint8_t reg);

bool MMC_init_chip(uint8_t outputRate = 2);
bool MMC_single_read();

bool Calibrate_Full_Soft_Iron(uint8_t num_seconds = 3, uint8_t num_timeout = 30);
void Apply_Cal_Matrix();

#endif /* MMC5983MA_DRIVER_H */