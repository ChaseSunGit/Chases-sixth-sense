/**
 * \file MEGASTRAIN5000.h
 * \brief Library for low level interfacing with the MPU6050. Use instead of arduino library if high speed application is needed.
 * TODO: 
 *
 * \author Chase Sun
 * \bug
 */

#ifndef MEGASTRAIN5000_H
#define MEGASTRAIN5000_H

// INCLUDES
#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h>

// CONSTANTS/MACROS

constexpr size_t NUM_MS5K = 5; //number of megastrain imus used

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

#define DATA_START_ICM      0x0B

// MLX 90393 registers




// CLASSES

struct MegaStrain_Data {
    uint8_t addr_IMU; //0x68 or 0x69
    uint8_t addr_MAG; //0x0E or 0x0F
    uint8_t addr_BMP; //0x46 or 0x47
    TwoWire* i2cBus; //Wire, Wire1, or Wire2

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

    float mx; //magnetometer x
    float my; //magnetometer x
    float mz; //magnetometer x

    Kalman kalmanRoll;//Roll kalman object
    Kalman kalmanPitch;//Pitch kalman object
    Kalman kalmanYaw; //Yaw kalman object
    float roll; //Euler angles
    float pitch;
    float yaw;

    float pressure; //BMP581 pressure

    bool valid_flag; //flag will be 1 if the IMU is being used
};

extern MegaStrain_Data MS5K_Data_Holder[NUM_MS5K];//5 imus, each will have an element of the struct


// FUNCTION PROTOTYPES
void MS5KWrite(int IMU_ID, uint8_t device_addr, uint8_t reg, uint8_t data);
uint8_t MS5KRead(int IMU_ID, uint8_t device_addr, uint8_t reg);

bool initICM(int IMU_ID, int reportFrequency);
bool initMLX(int IMU_ID, int reportFrequency);
bool initBMP(int IMU_ID, int reportFrequency);

void MegaStrain_bulk_initialization(int reportFrequency); // initialize all available ICM, MLX, and BMP available

void ICM_bulk_calibration(int num_samples, volatile bool& interrupt);
bool ICM_read_accel_gyro_lightweight(int IMU_ID, bool calibrationMode = false);
void ICM_factory_accel_calibration(int IMU_ID, int num_samples, volatile bool& interrupt);


bool MLX_read_mag_lightweight(int IMU_ID);
bool BMP_read_pressure_lightweight(int IMU_ID);


void read_all_MS5K(bool calibrationMode = false);
void Kalman_single_MS5K(int IMU_ID,float dt);
void Kalman_batch_MS5K();

#endif /* MEGASTRAIN5000_H */
