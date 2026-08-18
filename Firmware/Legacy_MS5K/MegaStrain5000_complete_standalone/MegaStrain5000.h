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

const int Moving_average_windowSize = 40;   // Number of samples in average for our moving aver


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

//The MLX combines the command and register. first half-byte is the register and the second half byte is the command
#define CMD_RESET_MLX 0xF0
#define CMD_SM_XYZ_MLX 0x3E // Single measurement,  0b 0011 1110 for xyz
#define CMD_RM_XYZ_MLX 0x4E // Read measurement,    0b 0100 1110 for xyz

#define CMD_WR_MLX 0x60 // Write to register,       0b 0110 0000 for an empty command (The command is the register address to write to)
#define CMD_RR_MLX 0x50 // Read register,           0b 0101 0000 for an empty command (The command is the register address to be read)

#define REG_0_ADDR_MLX 0x00
#define REG_1_ADDR_MLX 0x01
#define REG_2_ADDR_MLX 0x02

#define REG_0_CONF_MLX 0x7C // 0b 0 (z series) 111 (1x gain) 0000 (hall config) 0000000 (reserved) 0 (BIST)
#define REG_1_CONF_MLX 0x08 // 0b 00 (burst sel) 000000 (burst_data_rate) 0 (trig_int, 1 would be on int) 00 (Comm mode) 0 (WOC_DIFF) 1 (EXT-TRIG) 0 (TCMP_EN) 00 (BURST_SEL)
//Note: We keep the external triggger as zero for now
#define REG_2_CONF_MLX 0x05 // All zeros, OSR for temp and xyz, resolution, digital ilter all turned off

// Magnetometer hard/soft iron look up tables. These are only for 5 specific MLXs setup with the MF3000. You need to repeat the calibration procedure in MegaStrain5000_complete_standalone.ino to calibrate new sensors.
const float MLX_Hardiron[5][3] = {
    //Structured as [imu_id][axis], 0=x, 1=y, 2=z
    {22.4681, -42.4781, 25.7518},
    {16.9950, -80.4862, 1.4460},
    {19.5394, -38.9363, 20.3583},
    {22.4231, -45.1238, 2.8768},
    {31.6425, -49.5169, 38.4175}
};
const float MLX_Softiron[5][3] = {//THe softiron calibration is drastically simplified to basically a normalization value to bring the mag reading between -1 and 1
    //Structured as [imu_id][axis], 0=x, 1=y, 2=z
    {0.0217, 0.0212, 0.0208},
    {0.0183, 0.0196, 0.0172},
    {0.0194, 0.0207, 0.0191},
    {0.0194, 0.0200, 0.0189},
    {0.0212, 0.0205, 0.0211}
};


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
    float mx_max;
    float my_max;
    float mz_max;
    float mx_min;
    float my_min;
    float mz_min;


    float mx_array[Moving_average_windowSize];//Arrays for moving average
    float my_array[Moving_average_windowSize];
    float mz_array[Moving_average_windowSize];

    int ma_index_x; //index of the moving average filter
    int ma_index_y; //index of the moving average filter
    int ma_index_z; //index of the moving average filter


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
uint8_t MLX_transceive(int IMU_ID, uint8_t device_addr, uint8_t *txbuf, uint8_t txlen, uint8_t *rxbuf = NULL, uint8_t rxlen = 0, uint8_t interdelay = 10);
uint8_t MLX_writeRegister(int IMU_ID, uint8_t device_addr, uint8_t reg, uint8_t hbyte, uint8_t lbyte);
uint8_t MLX_readRegister(int IMU_ID, uint8_t device_addr, uint8_t reg, uint8_t *hbyte, uint8_t *lbyte);
uint8_t MS5KRead(int IMU_ID, uint8_t device_addr, uint8_t reg);

bool initICM(int IMU_ID, int reportFrequency);
bool initMLX(int IMU_ID);
bool initBMP(int IMU_ID);

void MegaStrain_bulk_initialization(int reportFrequency); // initialize all available ICM, MLX, and BMP available

void ICM_bulk_calibration(int num_samples, volatile bool& interrupt);
bool ICM_read_accel_gyro_lightweight(int IMU_ID, bool calibrationMode = false);
void ICM_factory_accel_calibration(int IMU_ID, int num_samples, volatile bool& interrupt);

void MLX_bulk_trigger_mag_lightweight();
uint8_t MLX_read_mag_lightweight(int IMU_ID);
bool BMP_read_pressure_lightweight(int IMU_ID);


void read_all_MS5K(bool calibrationMode = false);
void Kalman_single_MS5K(int IMU_ID,float dt,int mode = 0);
void Kalman_batch_MS5K(int mode = 0);

float moving_average(float *buffer, float new_val, int &ma_index);

#endif /* MEGASTRAIN5000_H */
