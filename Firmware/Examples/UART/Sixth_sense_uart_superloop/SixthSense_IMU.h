/**
 * \file SixthSense_IMU.h
 * \brief Unified wrapper for ICM42607 and MMC5983MA with x-io Fusion integration.
 */

#ifndef SIXTHSENSE_IMU_H
#define SIXTHSENSE_IMU_H

#include <Fusion.h> // x-io Technologies Fusion library https://github.com/xioTechnologies/Fusion

#include "ICM42607_Driver.h"
#include "MMC5983MA_Driver.h"

struct Fusion_Config_t {
      uint8_t 
};

class SixthSense_IMU {
public:
      // Constructor handles RTOS routing for both underlying sensors
      SixthSense_IMU(bool use_rtos, TaskHandle_t sensor_task);

      // Unified Initialization
      bool init(const ICM_Config_t &icm_cfg, const MMC_Config_t &mmc_cfg);

      // Unified Processing Loop (Call in superloop or RTOS task)
      bool processSensorData();

      // Pass-throughs for explicit Subclass Functionality
      bool calibrateAccel(int num_samples = 200);
      bool calibrateGyro(int num_samples = 200);
      bool calibrateMag(uint8_t num_seconds = 6, uint8_t num_timeout = 30);

      // Read settings
      void readICMconfig();
      void readMMCconfig();

      // Getters for Raw and Calibrated Device Data
      ICM_Data_t getICMData() const;
      MMC_Data_t getMMCData() const;

private:
      ICM42607 icm;
      MMC5983MA mmc;

      // x-io Fusion variables
      FusionAhrs ahrs;
      uint32_t last_time_micros;

      constexpr float GRAVITY_ICM = 9.81f; // Convert Gs to m/s^2
      constexpr float DEG2RAD_ICM = 0.01745329f; // Convert dps to rad/s
};

#endif /* SIXTHSENSE_IMU_H */