/**
 * \file MMC5983MA_Driver.h
 * \brief OOP Header file for driving the MMC5983MA Magnetometer
 */

#ifndef MMC5983MA_DRIVER_H
#define MMC5983MA_DRIVER_H

#include <Arduino.h>
#include <Math.h>
#include <String.h>
#include <vector>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "Sixth_sense_SPI.h"

// CONSTANTS/MACROS
#define MMC_BURST_LEN   8
#define COUNTS_PER_G_18 16384.0f // 18-bit counts per Gauss for ±2G range
#define NULL_FIELD_18   131072.0f // 18-bit null field value for sensitivity compensation

// SPI Pins
#define PIN_INT_MMC     4   
#define PIN_MISO_MMC    8   
#define PIN_MOSI_MMC    9   
#define PIN_SCLK_MMC    10  
#define PIN_CS_MMC      13  

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

#define MMC_SPI_READ_FLAG 0x80 
#define MMC_SPI_ADDR_MASK 0x3F 

// STRUCTURES
struct MMC_Data_t {
      float mag[3];
      float offset[3];
      float W[3][3];
      float mag_cal[3];
      bool newData;//Boolean to determine if the new data being read in is new or a repeat read of the old data
};

struct MMC_Config_t {
      uint8_t outputRate;     //1-7: 1 (1Hz), 2 (10Hz), 3 (20Hz), 4 (50Hz), 5 (100Hz), 6 (200Hz, default), 7(1000Hz)
      uint8_t bandwidth;      //1-4: 1 (100Hz, default), 2 (200Hz), 3 (400Hz), 4 (800Hz)
      uint16_t setFrequency;   //0-8 (measurements per set): 0: (disable autoset), 1 (1 sample), 2 (25), 3 (75), 4 (100), 5 (250), 6 (500), 7 (1000), 8 (2000 samples)
      bool chip_enable;       //0 - disable, 1 - enable
};

class MMC5983MA {
public:
      MMC5983MA(bool use_rtos, TaskHandle_t sensor_task);

      // Configuration tools
      bool read_config(MMC_Config_t &out_config);
      bool check_config_validity(const MMC_Config_t &config);
      static bool parse_config(const MMC_Config_t &config);

      //Initialization
      bool init_chip(const MMC_Config_t &config);
      //Single read
      bool single_read();
      
      // Calibration
      bool Calibrate_Full_Soft_Hard_Iron(uint8_t num_seconds = 6, uint8_t num_timeout = 30);
      
      MMC_Data_t getData() const { return data_holder; }
      float getODR() const {return ODR;}
      void force_data_ready() { new_data_ready = true; }

      static MMC5983MA* instance;
      static void IRAM_ATTR ISR_dataReady();
      static void IRAM_ATTR ISR_DMAcomplete_callback(spi_transaction_t *trans);

      static constexpr int freq_table[7] = { 1, 10, 20, 50, 100, 200, 1000 };
      static constexpr int bw_hz_table[4] = { 100, 200, 400, 800 };
      static constexpr float bw_time_table[4] = { 8.0f, 4.0f, 2.0f, 0.5f };
      static constexpr int set_samples_table[8] = { 1, 25, 75, 100, 250, 500, 1000, 2000 };

private:
      void Apply_Cal_Matrix();

      // Internal Math / Calibration Helpers
      bool Solve9x9(float A[9][9], float b[9], float x[9]);
      bool Invert3x3(const float A[3][3], float inv[3][3]);
      bool MatrixSqrt3x3(const float M[3][3], float W[3][3]);
      size_t MMC_Read_Block(uint8_t block_seconds, std::vector<float>& x_out, std::vector<float>& y_out, std::vector<float>& z_out);
      bool Evaluate_Calibration_Quality(const std::vector<float>& raw_x, const std::vector<float>& raw_y, const std::vector<float>& raw_z, const float offset[3], const float W[3][3]);

      SPI_DMA_Channel spi_dma;
      MMC_Data_t data_holder;
      
      int ODR; //output data rate

      volatile bool dma_in_progress;
      volatile bool new_data_ready;
      bool using_RTOS;
      TaskHandle_t SensorTaskHandle;

      uint32_t mag_last_measurement[3];//Used to track if a new measurement is made by seeing if it is the same as the last measurement
};

#endif /* MMC5983MA_DRIVER_H */