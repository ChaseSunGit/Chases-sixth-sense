/**
 * \file MPU6050_MF3000.h
 * \brief Library for low level interfacing with the MPU6050. Use instead of arduino library if high speed application is needed.
 * TODO: 
 *
 * \author Chase Sun
 * \bug
 */

#ifndef MPU6050_MF3000_H
#define MPU6050_MF3000_H

// INCLUDES
#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h>

// CONSTANTS/MACROS

constexpr size_t NUM_IMUS = 5;

// MPU6050 registers
#define PWR_MGMT_1     0x6B
#define WHO_AM_I       0x75
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define INT_ENABLE     0x38

// CLASSES

struct IMUData {
    uint8_t addr; //0x68 or 0x69
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
    Kalman kalmanRoll;//Roll kalman object
    Kalman kalmanPitch;//Pitch kalman object
    float roll; //Euler angles
    float pitch;
    float yaw;
    bool valid_flag; //flag will be 1 if the IMU is being used
};

extern IMUData IMU_Data_Holder[NUM_IMUS];//5 imus, each will have an element of the struct


// FUNCTION PROTOTYPES
void mpuWrite(int IMU_ID, uint8_t reg, uint8_t data);
uint8_t mpuRead(int IMU_ID, uint8_t reg);
bool initMPU6050(int IMU_ID, int reportFrequency);
void mpu6050_bulk_initialization(int reportFrequency);
bool enable_IMU_interrupt(int IMU_ID);
void mpu6050_bulk_calibration(int num_samples, volatile bool& interrupt);
bool mpu6050_read_accel_gyro_lightweight(int IMU_ID, bool calibrationMode = false);
void read_all_imus(bool calibrationMode = false);
void Kalman_single_imu(int IMU_ID,float dt);
void Kalman_batch_imu();
void mpu6050_factory_accel_calibration(int IMU_ID, int num_samples, volatile bool& interrupt);

#endif /* MPU6050_MF3000_H */
