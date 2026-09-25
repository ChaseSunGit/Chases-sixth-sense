/**
 * \file ICM42607_Driver.h
 * \brief Library for low level interfacing with the MPU6050. Use instead of arduino library if high speed application is needed.
 * TODO: 
 *
 * \author Chase Sun
 * \bug
 */

#ifndef ICM42607_DRIVER_H
#define ICM42607_DRIVER_H

// INCLUDES
#include <Arduino.h>
#include <Kalman.h>
#include <Preferences.h> //Used for saving to SPI flash memory
#include "driver/spi_master.h"
#include "esp_heap_caps.h"


// CONSTANTS/MACROS
// Read length: 1 byte address header (Echo when writing register) + 14 payload bytes (2 temp, 3 accel, 3 gyro) = 15 bytes total
#define ICM_BURST_LEN 15
#define Moving_average_windowSize 10

// SPI device using SPI_HOST2
extern spi_device_handle_t spi_ICM;

// DMA buffers for read and write sequences to be assigned to internal memory
extern uint8_t *dma_tx_ICM;
extern uint8_t *dma_rx_ICM;

// Persistent transaction descriptor for async DMA
extern spi_transaction_t dma_trans_ICM;
extern volatile bool dma_in_progress;
extern volatile bool new_data_ready;

extern bool IMU_first_read;


//SPI Pins
#define PIN_INT_ICM 7   // IMU Data Ready Interrupt
#define PIN_MISO    8   // Sensor SDO
#define PIN_MOSI    9   // Sensor SDI
#define PIN_SCLK    10  // SCLK
#define PIN_CS      11  // CS

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

#define DATA_START_ICM      0x09 // Start from temp

#define SPI_READ_FLAG       0x80 //For adding to 7-bit address to identify a read operation. A write will lead with a 0 so no operation required


// CLASSES

struct MAG_Data {

    int frequency; //Data rate setting for ICM

    float temp; //Temp of sensor for corrections

    float ax; // Acceleration X
    float ay; // Acceleration Y
    float az; // Acceleration Z
    float ax_offset; //Gyro x calibrated offset. These offsets are constant and are found through the factory calibration function and set manually
    float ay_offset; //Gyro y calibrated offset
    float az_offset; //Gyro z calibrated offset

    float gx; // Gyro X
    float gy; // Gyro Y
    float gz; // Gyro Z
    float gx_offset; //Gyro x calibrated offset
    float gy_offset; //Gyro y calibrated offset
    float gz_offset; //Gyro z calibrated offset

    Kalman kalmanRoll;//Roll kalman object
    Kalman kalmanPitch;//Pitch kalman object
    Kalman kalmanYaw; //Yaw kalman object
    float roll; //Euler angles
    float pitch;
    float yaw;

};

extern ICM_Data ICM_Data_Holder; //Holds data of the ICM readings


// FUNCTION PROTOTYPES
void IRAM_ATTR IMU_ISR_dataReady();
void IRAM_ATTR IMU_ISR_DMAcomplete_callback(spi_transaction_t *trans);

bool ICM_SPI_config();
bool ICM_DMA_config();
void ICM_write_reg(uint8_t reg, uint8_t data);
uint8_t ICM_read_reg(uint8_t reg);

bool ICM_init_chip(uint8_t outputRate);

//void ICM_calibration(int num_samples);
bool ICM_single_read();
//void ICM_factory_accel_calibration(int IMU_ID, int num_samples, volatile bool& interrupt);

void ICM_Kalman_fusion(float dt,int mode = 0);

float moving_average(float *buffer, float new_val, int &ma_index);

bool ICM_accel_calib(int num_samples);

bool ICM_gyro_calib(int num_samples);

#endif /* ICM42607_DRIVER_H */
