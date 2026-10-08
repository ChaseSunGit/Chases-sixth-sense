/**
 * \file SixthSense_Config.h
 * \brief Configuration and calibration manager for the SixthSense IMU over NVS Flash.
 */

#ifndef SIXTHSENSE_CONFIG_H
#define SIXTHSENSE_CONFIG_H

#include <Arduino.h>
#include <nvs_flash.h>
#include <nvs.h>
#include "SixthSense_IMU.h"

class SixthSense_Config {
public:
      SixthSense_Config(SixthSense_IMU* imu_ptr);

      // Initialize NVS and load settings/calibration into the provided structs
      bool begin(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);

      // Non-blocking check for the "config" command on Serial 
      void checkUART(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);

private:
      SixthSense_IMU* imu;
      nvs_handle_t my_handle;
      
      // NVS Read/Write wrappers
      bool loadConfigs(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);
      bool saveConfigs(const ICM_Config_t &icm_cfg, const MMC_Config_t &mmc_cfg, const Fusion_Config_t &fusion_cfg);
      
      bool loadCalibration();
      bool saveCalibration();

      // Blocking configuration menu loop
      void enterConfigMode(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);
      void printMenu();
      void parseCommand(String cmd, ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);
};

#endif // SIXTHSENSE_CONFIG_H