/**
 * \file ICM42607_Driver.h
 * \brief Library for low level interfacing with the ICM42607
 * TODO: 
 *
 * \author Chase Sun
 * \bug
 */

#ifndef ICM42607_Driver_H
#define ICM42607_Driver_H

// INCLUDES
#include <Arduino.h>
#include <Kalman.h>


// ICM 42607 registers
#define WHO_AM_I_ICM        0x75

#define INT_CONFIG_ICM      0x06
#define INT_CONFIG0_ICM     0x04 //On MREG1
#define INT_SOURCE0_ICM     0x2B // Source 0 chooses the source of interrupt for int1
#define INT_SOURCE3_ICM     0x2D // Source 3 chooses the source of interrupt for int 2

#define PWR_MGMT0_ICM       0x1F
#define GYRO_CONFIG0_ICM    0x20
#define ACCEL_CONFIG0_ICM   0x21
#define GYRO_CONFIG1_ICM    0x23
#define ACCEL_CONFIG1_ICM   0x24

#define FIFO_CONFIG1_ICM    0x28
#define FIFO_CONFIG2_ICM    0x29
#define FIFO_CONFIG3_ICM    0x2A

#define FIFO_COUNTH_ICM     0x3D
#define FIFO_COUNTL_ICM     0x3E
#define FIFO_DATA_ICM       0x3F
#define SIGNAL_PATH_RESET_ICM 0x02

//MREG1 for FIFO config 5
#define FIFO_CONFIG5_ICM    0x01
//MREG1 access procedure
#define BLK_SEL_W_ICM       0x79
#define MADDR_W_ICM         0x7A
#define M_W_ICM             0x7B

//Slew rate
#define DRIVE_CONFIG2_ICM   0x04
#define DATA_START_ICM      0x0B


// STRUCTURES

struct ICM_Data {
    uint8_t addr_IMU; //0x68 or 0x69

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

    bool valid_flag; //flag will be 1 if the IMU is being used
};


// FUNCTION PROTOTYPES
void ICMWrite(uint8_t reg, uint8_t data);
uint8_t ICMRead(uint8_t reg);

bool initICM(int reportFrequency);

void ICM_calibration(int num_samples);
bool ICM_read_accel_gyro();
void ICM_factory_accel_calibration(int num_samples);


void Kalman_ICM(float dt,int mode = 0);


#endif /* ICM42607_Driver_H */
