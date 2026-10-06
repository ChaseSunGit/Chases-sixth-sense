/**
 * \file SixthSense_IMU.cpp
 * \brief Implementation of the unified SixthSense_IMU wrapper.
 */

#include "SixthSense_IMU.h"

// Initialize both objects via initializer list passing RTOS settings[cite: 1, 3]
SixthSense_IMU::SixthSense_IMU(bool use_rtos, TaskHandle_t sensor_task) 
            : icm(use_rtos, sensor_task), 
              mmc(use_rtos, sensor_task), 
              last_time_micros(0) {
      
      // Initialize Fusion structures
      FusionOffsetInitialise(&offset, 2000); 
      FusionAhrsInitialise(&ahrs);
}

bool SixthSense_IMU::init(const ICM_Config_t &icm_cfg, const MMC_Config_t &mmc_cfg) {
    bool icm_ready = icm.init_chip(icm_cfg);
    bool mmc_ready = mmc.init_chip(mmc_cfg);

    if (icm_ready) {
        // Configure x-io AHRS settings based on initialization
        FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * (int)icm.getODR() 
        };
        FusionAhrsSetSettings(&ahrs, &settings);
        FusionOffsetInitialise(&offset, (int)icm.getODR());
    }

    last_time_micros = micros();
    return (icm_ready && mmc_ready);
}

bool SixthSense_IMU::processSensorData() {
    // Attempt single_reads. These unpack the DMA buffers if data is flagged ready[cite: 1, 3]
    bool icm_new = icm.single_read();
    bool mmc_new = mmc.single_read();

    // If no new accelerometer/gyro data, skip fusion update for this cycle
    if (!icm_new) return false;

    // Time delta integration
    uint32_t current_time = micros();
    float delta_time = (current_time - last_time_micros) / 1000000.0f;
    last_time_micros = current_time;

    ICM_Data_t icm_data = icm.getData();
    MMC_Data_t mmc_data = mmc.getData();

    // x-io expects g and dps. ICM provides m/s^2 and rad/s[cite: 1].
    FusionVector gyroscope = {
        .axis = {
            icm_data.gyro_cal[0] * RAD_TO_DPS,
            icm_data.gyro_cal[1] * RAD_TO_DPS,
            icm_data.gyro_cal[2] * RAD_TO_DPS
        }
    };

    FusionVector accelerometer = {
        .axis = {
            icm_data.accel_cal[0] * MS2_TO_G,
            icm_data.accel_cal[1] * MS2_TO_G,
            icm_data.accel_cal[2] * MS2_TO_G
        }
    };

    // MMC5983MA provides Gauss directly via single_read[cite: 3].
    FusionVector magnetometer = {
        .axis = {
            mmc_data.mx_cal,
            mmc_data.my_cal,
            mmc_data.mz_cal
        }
    };

    // Apply offset compensation and run the fusion algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
    
    if (mmc_new) {
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta_time);
    } else {
        // If mag data isn't fresh (e.g. mag ODR is lower than IMU ODR), run without mag update
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_time);
    }

    return true;
}

// Pass-through wrapper functions
bool SixthSense_IMU::calibrateAccel(int num_samples) {
      return icm.accel_calib(num_samples);
}

bool SixthSense_IMU::calibrateGyro(int num_samples) {
      return icm.gyro_calib(num_samples);
}

bool SixthSense_IMU::calibrateMag(uint8_t num_seconds, uint8_t num_timeout) {
      return mmc.Calibrate_Full_Soft_Hard_Iron(num_seconds, num_timeout);
}

ICM_Data_t SixthSense_IMU::getICMData() const {
      return icm.getData();
}

MMC_Data_t SixthSense_IMU::getMMCData() const {
      return mmc.getData();
}

void SixthSense_IMU:readICMconfig() {
      ICM_Config_t read_config;
      return icm.read_config(read_config);
}

void SixthSense_IMU:readMMCconfig() {
      MMC_Config_t read_config;
      return mmc.read_config(read_config);
}

FusionEuler SixthSense_IMU::getEulerAngles() const {
    return FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
}

FusionQuaternion SixthSense_IMU::getQuaternion() const {
    return FusionAhrsGetQuaternion(&ahrs);
}