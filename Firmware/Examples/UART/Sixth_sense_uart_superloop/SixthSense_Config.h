/**
 * \file SixthSense_Config.h
 * \brief Configuration and calibration manager for the SixthSense IMU over NVS Flash.
 *
 * \author    Chase Sun
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

      // Initialize NVS. If no data exists, it generates defaults and writes them.
      bool begin(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);

      // Lifecycle & Task Management
      bool RTOS_startConfigTask(BaseType_t core_id = 1, UBaseType_t priority = 1);//Low priority task used to monitor UART function
      void RTOS_stopConfigTask();
      // Non-blocking check for the "config" command on Serial
      void checkUART();

private:
      SixthSense_IMU* imu;
      nvs_handle_t imu_config_handle;
      
      // NVS Read/Write wrappers
      bool loadConfigs(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);
      bool saveConfigs(const ICM_Config_t &icm_cfg, const MMC_Config_t &mmc_cfg, const Fusion_Config_t &fusion_cfg);
      
      bool loadCalibration();
      bool saveCalibration();

      void displayCalibration();

      // Resets configs to default, zeros calibrations, and saves to NVS
      void factoryReset(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);

      // Blocking configuration menu loops
      void enterConfigMode();
      void printMainMenu();
      
      // Sub-menus for specific configuration blocks
      void icmMenu(ICM_Config_t &icm_cfg);
      void magMenu(MMC_Config_t &mmc_cfg);
      void fusionMenu(Fusion_Config_t &fusion_cfg);

      // FreeRTOS Task Handle & Trampoline
      TaskHandle_t config_task_handle = NULL;
      static void Config_Task_Trampoline(void *pvParameters);
};

#endif // SIXTHSENSE_CONFIG_H