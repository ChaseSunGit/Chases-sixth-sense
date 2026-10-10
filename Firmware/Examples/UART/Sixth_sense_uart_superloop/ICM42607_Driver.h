/**
 * \file ICM42607_Driver.h
 * \brief Header file for driving the ICM42607 IMU (OOP Version)
 */

#ifndef ICM42607_DRIVER_H
#define ICM42607_DRIVER_H

#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "SixthSense_SPI.h"
#include "MMC5983MA_Driver.h" 

// Constants / Macros
#define ICM_BURST_LEN 15
#define Moving_average_windowSize 10

// SPI Pins
#define PIN_INT_ICM     7
#define PIN_MISO_ICM    8
#define PIN_MOSI_ICM    9
#define PIN_SCLK_ICM    10
#define PIN_CS_ICM      11

// ICM 42607 registers
#define WHO_AM_I_ICM        0x75
#define INT_CONFIG_ICM      0x06
#define INT_SOURCE0_ICM     0x2B 
#define INT_SOURCE3_ICM     0x2D 
#define PWR_MGMT0_ICM       0x1F
#define GYRO_CONFIG0_ICM    0x20
#define ACCEL_CONFIG0_ICM   0x21
#define GYRO_CONFIG1_ICM    0x23
#define ACCEL_CONFIG1_ICM   0x24
#define TEMP_CONFIG0_ICM    0x22
#define DATA_START_ICM      0x09 

#define SPI_READ_FLAG       0x80 

//Structures
struct ICM_Data_t {
      float temp; 
      float accel[3]; 
      float accel_cal[3]; 
      float gyro[3];
      float gyro_cal[3];
};

struct ICM_Cal_t {
      float accel_offset[3];
      float gyro_offset[3];
};

struct ICM_Config_t {
      uint8_t outputRate;     //1-8: 1 (12.5Hz), 2 (25Hz), 3 (50Hz), 4 (100Hz), 5 (200Hz, default), 6 (400Hz), 7(800Hz), 8(1600Hz)
      uint8_t accel_bw;       //1-8: 1 (16Hz), 2 (25Hz), 3 (34Hz), 4 (53Hz), 5 (73Hz, default), 6 (121Hz), 7(180Hz), 8(Bypassed)
      uint8_t gyro_bw;        //1-8: 1 (16Hz), 2 (25Hz), 3 (34Hz), 4 (53Hz), 5 (73Hz, default), 6 (121Hz), 7(180Hz), 8(Bypassed)
      uint8_t accel_range;    //1-4: 1 (±2g), 2 (±4g), 3 (±8g, default), 4 (±16g)
      uint8_t gyro_range;     //1-4: 1 (±250dps), 2 (±500dps), 3 (±1000dps, default), 4 (±2000dps)    
      bool chip_enable;       //Switch to toggle ICM: 0 - disable, 1 - enable
};

class ICM42607 {
public:
      //Constructor
      ICM42607();

      //Config helper tools
      bool read_config(ICM_Config_t &out_config);
      bool check_config_validity(const ICM_Config_t &config);
      static void parse_config(const ICM_Config_t &config);

      //Initalization
      bool init_chip(const ICM_Config_t &config);

      //Single read
      bool single_read();

      //Apply calibration constants to offset measurements
      void apply_calibration();

      //Apply temp correction
      void temp_correct();
      
      //Sensor individual calibration
      bool accel_calib(int num_samples = 2000);//Calibrated once and then store
      bool gyro_calib(int num_samples = 2000);//Calibrate at every startup
      ICM_Cal_t getCal() const {return calibration_const;}
      void setCal(const ICM_Cal_t &cal_const) {calibration_const = cal_const;}

      //Get data for external reads
      ICM_Data_t getData() const { return data_holder;} 
      float getODR() const {return ODR;}

      //Force a data ready without interrupt - used for calibration
      void force_data_ready() {new_data_ready = true;}//Function to force a read by artificially raising the data ready flag

      static constexpr float odr_table[8] = {12.5f, 25.0f, 50.0f, 100.0f, 200.0f, 400.0f, 800.0f, 1600.0f};
      static constexpr int bw_table[8] = {16, 25, 34, 53, 73, 121, 180, 0};
      static constexpr int accel_range_table[4] = {2, 4, 8, 16};
      static constexpr int gyro_range_table[4]  = {250, 500, 1000, 2000};
      //Expose the lookup tables for external reference in code

private:
      //SPI functions
      void write_reg(uint8_t reg, uint8_t data);
      uint8_t read_reg(uint8_t reg);

      //Chip enable
      bool is_enabled;

      //SPI handle
      spi_device_handle_t spi_handle;

      //Private data holder
      ICM_Data_t data_holder;
      ICM_Cal_t calibration_const;
      
      //Booleans for handling data ready
      volatile bool new_data_ready;

      //Internal stored config values
      int ODR; //output data rate
      float accel_conversion; 
      float gyro_conversion;

      //Full range config
      static constexpr float ACCEL_LSB_PER_G[4] = {2048.0, 4096.0, 8192.0, 16384.0};
      static constexpr float GYRO_LSB_PER_DPS[4] = {16.384, 32.768, 65.536, 131.072};

};
#endif /* ICM42607_DRIVER_H */