/**
 * \file SixthSense_Config.cpp
 * \brief Implementation of the configuration manager.
 */

#include "SixthSense_Config.h"

SixthSense_Config::SixthSense_Config(SixthSense_IMU* imu_ptr) : imu(imu_ptr) {}

bool SixthSense_Config::begin(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      // Initialize NVS
      esp_err_t err = nvs_flash_init();
      if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
      }
      ESP_ERROR_CHECK(err);

      // Load configs or set defaults
      if (!loadConfigs(icm_cfg, mmc_cfg, fusion_cfg)) {
            Serial.println("[NVS] No saved configs found. Loading defaults.");
            imu->returnDefaultConfig(icm_cfg, mmc_cfg, fusion_cfg);
      } else {
            Serial.println("[NVS] Configs loaded from flash.");
      }
      return true;
}

void SixthSense_Config::checkUART(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input.equalsIgnoreCase("config")) {
                  enterConfigMode(icm_cfg, mmc_cfg, fusion_cfg);
            }
      }
}

void SixthSense_Config::enterConfigMode(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      Serial.println("\n=== ENTERING CONFIGURATION MODE ===");
      
      // Suspend RTOS task to stop data processing while interacting
      imu->suspendTask(); 
      
      printMenu();

      bool inConfig = true;
      while (inConfig) {
            if (Serial.available()) {
                  String cmd = Serial.readStringUntil('\n');
                  cmd.trim();
                  
                  if (cmd.equalsIgnoreCase("exit")) {
                  Serial.println("Exiting config mode. Applying and saving changes...");
                  saveConfigs(icm_cfg, mmc_cfg, fusion_cfg);
                  
                  // Re-initialize sensors with new structs
                  imu->sensor_init(icm_cfg, mmc_cfg, fusion_cfg);
                  
                  // Apply loaded/updated calibrations
                  loadCalibration(); 
                  
                  inConfig = false;
                  } else if (cmd.equalsIgnoreCase("menu")) {
                  printMenu();
                  } else {
                  parseCommand(cmd, icm_cfg, mmc_cfg, fusion_cfg);
                  }
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // Prevent watchdog starvation
      }

      Serial.println("=== RESUMING SENSOR OPERATIONS ===");
      imu->resumeTask();
}

void SixthSense_Config::printMenu() {
      Serial.println("Available Commands:");
      Serial.println("  set icm_odr <1-8>         - Set ICM Output Data Rate");
      Serial.println("  set icm_accel_range <1-4> - Set ICM Accel Range");
      Serial.println("  set mmc_odr <1-7>         - Set MMC Output Data Rate");
      Serial.println("  cal accel                 - Calibrate Accelerometer");
      Serial.println("  cal gyro                  - Calibrate Gyroscope");
      Serial.println("  cal mag                   - Calibrate Magnetometer");
      Serial.println("  save_cal                  - Save current calibration to flash");
      Serial.println("  exit                      - Save configs, re-init, and exit");
}

void SixthSense_Config::parseCommand(String cmd, ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      int spaceIdx = cmd.indexOf(' ');
      String action = (spaceIdx == -1) ? cmd : cmd.substring(0, spaceIdx);
      String target = (spaceIdx == -1) ? "" : cmd.substring(spaceIdx + 1);

      if (action.equalsIgnoreCase("set")) {
            int valIdx = target.indexOf(' ');
            if (valIdx == -1) { Serial.println("Invalid set command. Format: set <variable> <value>"); return; }
            
            String var = target.substring(0, valIdx);
            int value = target.substring(valIdx + 1).toInt();

            if (var.equalsIgnoreCase("icm_odr")) { icm_cfg.outputRate = value; Serial.printf("ICM ODR set to %d\n", value); }
            else if (var.equalsIgnoreCase("icm_accel_range")) { icm_cfg.accel_range = value; Serial.printf("ICM Accel Range set to %d\n", value); }
            else if (var.equalsIgnoreCase("mmc_odr")) { mmc_cfg.outputRate = value; Serial.printf("MMC ODR set to %d\n", value); }
            else { Serial.println("Unknown variable."); }
            
      } else if (action.equalsIgnoreCase("cal")) {
            if (target.equalsIgnoreCase("accel")) {
                  Serial.println("Calibrating Accel...");
                  imu->calibrateAccel(200);
            } else if (target.equalsIgnoreCase("gyro")) {
                  Serial.println("Calibrating Gyro...");
                  imu->calibrateGyro(200);
            } else if (target.equalsIgnoreCase("mag")) {
                  Serial.println("Calibrating Mag...");
                  imu->calibrateMag(6, 30);
            }
      } else if (action.equalsIgnoreCase("save_cal")) {
            saveCalibration();
      } else {
            Serial.println("Unknown command. Type 'menu' for options.");
      }
}

bool SixthSense_Config::loadConfigs(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      if (nvs_open("SixthSense", NVS_READONLY, &my_handle) != ESP_OK) return false;
      
      size_t len = sizeof(ICM_Config_t);
      bool success = true;
      
      if (nvs_get_blob(my_handle, "icm_cfg", &icm_cfg, &len) != ESP_OK) success = false;
      
      len = sizeof(MMC_Config_t);
      if (nvs_get_blob(my_handle, "mmc_cfg", &mmc_cfg, &len) != ESP_OK) success = false;
      
      len = sizeof(Fusion_Config_t);
      if (nvs_get_blob(my_handle, "fus_cfg", &fusion_cfg, &len) != ESP_OK) success = false;
      
      nvs_close(my_handle);
      return success;
}

bool SixthSense_Config::saveConfigs(const ICM_Config_t &icm_cfg, const MMC_Config_t &mmc_cfg, const Fusion_Config_t &fusion_cfg) {
      if (nvs_open("SixthSense", NVS_READWRITE, &my_handle) != ESP_OK) return false;
      
      nvs_set_blob(my_handle, "icm_cfg", &icm_cfg, sizeof(ICM_Config_t));
      nvs_set_blob(my_handle, "mmc_cfg", &mmc_cfg, sizeof(MMC_Config_t));
      nvs_set_blob(my_handle, "fus_cfg", &fusion_cfg, sizeof(Fusion_Config_t));
      
      nvs_commit(my_handle);
      nvs_close(my_handle);
      return true;
}

bool SixthSense_Config::saveCalibration() {
      if (nvs_open("SixthSense", NVS_READWRITE, &my_handle) != ESP_OK) return false;

      // Extract current calibration arrays from the sensor wrappers
      ICM_Data_t icmData = imu->getICMData();
      MMC_Data_t mmcData = imu->getMMCData();

      ICM_Cal_t icmCal;
      MMC_Cal_t mmcCal;

      memcpy(icmCal.accel_offset, icmData.accel_offset, sizeof(icmCal.accel_offset));
      memcpy(icmCal.gyro_offset, icmData.gyro_offset, sizeof(icmCal.gyro_offset));
      
      memcpy(mmcCal.offset, mmcData.offset, sizeof(mmcCal.offset));
      memcpy(mmcCal.W, mmcData.W, sizeof(mmcCal.W));

      nvs_set_blob(my_handle, "icm_cal", &icmCal, sizeof(ICM_Cal_t));
      nvs_set_blob(my_handle, "mmc_cal", &mmcCal, sizeof(MMC_Cal_t));
      
      nvs_commit(my_handle);
      nvs_close(my_handle);
      
      Serial.println("Calibration matrices saved to flash.");
      return true;
}

bool SixthSense_Config::loadCalibration() {
      if (nvs_open("SixthSense", NVS_READONLY, &my_handle) != ESP_OK) return false;

      ICM_Cal_t icmCal;
      MMC_Cal_t mmcCal;
      size_t len = sizeof(ICM_Cal_t);
      
      if (nvs_get_blob(my_handle, "icm_cal", &icmCal, &len) == ESP_OK) {
            imu->setICMCalibration(icmCal.accel_offset, icmCal.gyro_offset);
      }
      
      len = sizeof(MMC_Cal_t);
      if (nvs_get_blob(my_handle, "mmc_cal", &mmcCal, &len) == ESP_OK) {
            imu->setMMCCalibration(mmcCal.offset, mmcCal.W);
      }
      
      nvs_close(my_handle);
      return true;
}