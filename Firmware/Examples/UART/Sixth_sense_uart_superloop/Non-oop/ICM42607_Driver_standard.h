/**
 * \file ICM42607_Driver.h
 * \brief Header file for driving the ICM42607 IMU
 *
 * \author Chase Sun
 */

#ifndef ICM42607_DRIVER_H
#define ICM42607_DRIVER_H

// INCLUDES
#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "Sixth_sense_SPI.h"

#include "MMC5983MA_Driver.h" //This is done to allow the ICM's interrupt to drive the MMC's interrupt to unify sampling frequency


// CONSTANTS/MACROS
// Read length: 1 byte address header (Echo when writing register) + 14 payload bytes (2 temp, 3 accel, 3 gyro) = 15 bytes total
#define ICM_BURST_LEN 15
#define Moving_average_windowSize 10

extern SPI_DMA_Channel SPI_DMA_ICM;

extern volatile bool dma_in_progress_ICM;
extern volatile bool new_data_ready_ICM;

extern bool IMU_first_read;


//SPI Pins
#define PIN_INT_ICM     7   // IMU Data Ready Interrupt
#define PIN_MISO_ICM    8   // Sensor SDO
#define PIN_MOSI_ICM    9   // Sensor SDI
#define PIN_SCLK_ICM    10  // SCLK
#define PIN_CS_ICM      11  // CS

// ICM 42607 registers
#define WHO_AM_I_ICM        0x75

#define INT_CONFIG_ICM      0x06
#define INT_SOURCE0_ICM     0x2B // Source 0 chooses the source of interrupt for int1
#define INT_SOURCE3_ICM     0x2D // Source 3 chooses the source of interrupt for int 2

#define PWR_MGMT0_ICM       0x1F
#define GYRO_CONFIG0_ICM    0x20
#define ACCEL_CONFIG0_ICM   0x21
#define GYRO_CONFIG1_ICM    0x23
#define ACCEL_CONFIG1_ICM   0x24
#define TEMP_CONFIG0_ICM    0x22

#define DATA_START_ICM      0x09 // Start from temp

#define SPI_READ_FLAG       0x80 //For adding to 7-bit address to identify a read operation. A write will lead with a 0 so no operation required

//Conversion constants
constexpr float GRAV = 9.81f;
constexpr float DEG2RAD = 0.01745329f;

// CLASSES

struct ICM_Data_t {
      //Stored key config values
      int frequency; //Data rate setting for ICM
      float accel_conversion; //Conversion constant based on set full scale range
      float gyro_conversion;  //Conversion constant based on set full scale range

      float temp; //Temp of sensor for corrections

      float accel[3]; // Acceleration XYZ
      float accel_offset[3]; //Accelerometer offset XYZ
      float accel_cal[3]; //Calibrated acceleration XYZ

      float gyro[3];
      float gyro_offset[3];
      float gyro_cal[3];
};

struct ICM_Config_t {
      uint8_t outputRate;     //1-8: 1 (12.5Hz), 2 (25Hz), 3 (50Hz), 4 (100Hz), 5 (200Hz, default), 6 (400Hz), 7(800Hz), 8(1600Hz)
      uint8_t accel_bw;       //1-8: 1 (16Hz), 2 (25Hz), 3 (34Hz), 4 (53Hz), 5 (73Hz, default), 6 (121Hz), 7(180Hz), 8(Bypassed)
      uint8_t gyro_bw;        //1-8: 1 (16Hz), 2 (25Hz), 3 (34Hz), 4 (53Hz), 5 (73Hz, default), 6 (121Hz), 7(180Hz), 8(Bypassed)
      uint8_t accel_range;    //1-4: 1 (±2g), 2 (±4g), 3 (±8g, default), 4 (±16g)
      uint8_t gyro_range;     //1-4: 1 (±250dps), 2 (±500dps), 3 (±1000dps, default), 4 (±2000dps)
}

extern ICM_Data_t ICM_Data_Holder; //Holds data of the ICM readings
extern bool using_RTOS; //Boolean to determine if RTOS is used. 0 represents superloop and 1 represents RTOS operation
extern TaskHandle_t SensorTaskHandle;

// FUNCTION PROTOTYPES
void IRAM_ATTR ICM_ISR_dataReady();
void IRAM_ATTR ICM_ISR_DMAcomplete_callback(spi_transaction_t *trans);

void ICM_write_reg(uint8_t reg, uint8_t data);
uint8_t ICM_read_reg(uint8_t reg);

bool ICM_init_chip(uint8_t outputRate = 2);

bool ICM_single_read();

void ICM_apply_calibration();

float moving_average(float *buffer, float new_val, int &ma_index);

bool ICM_accel_calib(int num_samples = 200);

bool ICM_gyro_calib(int num_samples = 200);

#endif /* ICM42607_DRIVER_H */
