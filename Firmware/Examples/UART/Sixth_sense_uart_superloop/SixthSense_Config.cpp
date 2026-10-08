/**
 * \file SixthSense_Config.cpp
 * \brief Implementation of the configuration manager with sub-menus.
 */

#include "SixthSense_Config.h"

SixthSense_Config::SixthSense_Config(SixthSense_IMU* imu_ptr) : imu(imu_ptr) {}

bool SixthSense_Config::begin(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      // Initialize NVS
      esp_err_t err = nvs_flash_init();
      if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
      }
      ESP_ERROR_CHECK(err);

      // Attempt to load. If either fails, it's the first time running (or memory was wiped)
      bool configsLoaded = loadConfigs(icm_cfg, mmc_cfg, fusion_cfg);
      bool calsLoaded = loadCalibration();

      if (!configsLoaded || !calsLoaded) {
            Serial.println("[NVS] No saved configurations found. Performing initial setup...");
            factoryReset(icm_cfg, mmc_cfg, fusion_cfg);
      } else {
            Serial.println("[NVS] Configurations and calibrations successfully loaded from flash.");
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

void SixthSense_Config::factoryReset(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      Serial.println("[Config] Generating default configurations...");
      imu->returnDefaultConfig(icm_cfg, mmc_cfg, fusion_cfg);

      // Zero out IMU calibrations (using Identity Matrix for Magnetometer Soft Iron)
      ICM_Cal_t icm_cal = { {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f} };
      MMC_Cal_t mmc_cal = { 
            {0.0f, 0.0f, 0.0f}, 
            { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} } 
      };

      imu->setICMCal(icm_cal);
      imu->setMMCCal(mmc_cal);

      // Write to memory
      saveConfigs(icm_cfg, mmc_cfg, fusion_cfg);
      saveCalibration();

      Serial.println("[Config] Factory reset complete. Defaults committed to flash.");
}

void SixthSense_Config::enterConfigMode(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      Serial.println("\n=== ENTERING CONFIGURATION MODE ===");
      imu->RTOS_suspendReading(); // Halt background sensor tasks
      
      printMainMenu();

      bool inConfig = true;
      while (inConfig) {
            if (Serial.available()) {
                  String cmd = Serial.readStringUntil('\n');
                  cmd.trim();
                  cmd.toLowerCase();
                  
                  if (cmd == "exit") {
                  Serial.println("Exiting... Saving and re-initializing sensors.");
                  saveConfigs(icm_cfg, mmc_cfg, fusion_cfg);
                  imu->sensor_init(icm_cfg, mmc_cfg, fusion_cfg);
                  inConfig = false;
                  } else if (cmd == "menu") {
                  printMainMenu();
                  } else if (cmd == "accel") {
                  accelMenu(icm_cfg);
                  } else if (cmd == "gyro") {
                  gyroMenu(icm_cfg);
                  } else if (cmd == "mag") {
                  magMenu(mmc_cfg);
                  } else if (cmd == "fusion") {
                  fusionMenu(fusion_cfg);
                  } else if (cmd == "cal_accel") {
                  Serial.println("Calibrating Accel...");
                  imu->calibrateAccel(200);
                  } else if (cmd == "cal_gyro") {
                  Serial.println("Calibrating Gyro...");
                  imu->calibrateGyro(200);
                  } else if (cmd == "cal_mag") {
                  Serial.println("Calibrating Mag...");
                  imu->calibrateMag(6, 30);
                  } else if (cmd == "save_cal") {
                  saveCalibration();
                  } else if (cmd == "factory_reset") {
                  factoryReset(icm_cfg, mmc_cfg, fusion_cfg);
                  Serial.println("Reset applied. You can edit further or type 'exit' to deploy.");
                  } else {
                  Serial.println("Unknown command. Type 'menu' for options.");
                  }
            }
            vTaskDelay(pdMS_TO_TICKS(10)); 
      }

      Serial.println("=== RESUMING SENSOR OPERATIONS ===");
      imu->RTOS_resumeReading();
}

void SixthSense_Config::printMainMenu() {
      Serial.println("\n--- MAIN MENU ---");
      Serial.println("Type a category to enter its sub-menu:");
      Serial.println("  accel         - Accelerometer Settings (ODR, Range, BW, Enable)");
      Serial.println("  gyro          - Gyroscope Settings (Range, BW)");
      Serial.println("  mag           - Magnetometer Settings (ODR, BW, Set Freq, Enable)");
      Serial.println("  fusion        - Sensor Fusion Settings (Gains, Rejections, Axis)");
      Serial.println("\nDirect Commands:");
      Serial.println("  cal_accel     - Calibrate Accelerometer");
      Serial.println("  cal_gyro      - Calibrate Gyroscope");
      Serial.println("  cal_mag       - Calibrate Magnetometer");
      Serial.println("  save_cal      - Save current calibration to flash");
      Serial.println("  factory_reset - Wipe flash and load defaults");
      Serial.println("  exit          - Save all changes, apply, and resume operation");
}

// --------------------------------------------------------
// SUB-MENUS
// --------------------------------------------------------

void SixthSense_Config::accelMenu(ICM_Config_t &icm_cfg) {
      Serial.println("\n--- ACCEL SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set odr 5");
      Serial.println("Params: odr (1-8), range (1-4), bw (1-8), enable (0-1)");
      Serial.println("Type 'back' to return to main menu.");

      while(true) {
            if (Serial.available()) {
                  String cmd = Serial.readStringUntil('\n'); cmd.trim(); cmd.toLowerCase();
                  if (cmd == "back") { printMainMenu(); return; }
                  
                  if (cmd.startsWith("set ")) {
                  int spaceIdx = cmd.lastIndexOf(' ');
                  String param = cmd.substring(4, spaceIdx);
                  int val = cmd.substring(spaceIdx + 1).toInt();

                  if (param == "odr") { icm_cfg.outputRate = val; Serial.printf("ICM ODR set to %d\n", val); }
                  else if (param == "range") { icm_cfg.accel_range = val; Serial.printf("Accel Range set to %d\n", val); }
                  else if (param == "bw") { icm_cfg.accel_bw = val; Serial.printf("Accel BW set to %d\n", val); }
                  else if (param == "enable") { icm_cfg.chip_enable = val; Serial.printf("ICM Enable set to %d\n", val); }
                  else Serial.println("Unknown parameter.");
                  }
            }
            vTaskDelay(10);
      }
      }

      void SixthSense_Config::gyroMenu(ICM_Config_t &icm_cfg) {
      Serial.println("\n--- GYRO SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set range 3");
      Serial.println("Params: range (1-4), bw (1-8)");
      Serial.println("Note: Gyro ODR is shared with Accel ODR. Type 'back' to return.");

      while(true) {
            if (Serial.available()) {
                  String cmd = Serial.readStringUntil('\n'); cmd.trim(); cmd.toLowerCase();
                  if (cmd == "back") { printMainMenu(); return; }
                  
                  if (cmd.startsWith("set ")) {
                  int spaceIdx = cmd.lastIndexOf(' ');
                  String param = cmd.substring(4, spaceIdx);
                  int val = cmd.substring(spaceIdx + 1).toInt();

                  if (param == "range") { icm_cfg.gyro_range = val; Serial.printf("Gyro Range set to %d\n", val); }
                  else if (param == "bw") { icm_cfg.gyro_bw = val; Serial.printf("Gyro BW set to %d\n", val); }
                  else Serial.println("Unknown parameter.");
                  }
            }
            vTaskDelay(10);
      }
}

void SixthSense_Config::magMenu(MMC_Config_t &mmc_cfg) {
      Serial.println("\n--- MAG SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set odr 6");
      Serial.println("Params: odr (1-7), bw (1-4), setfreq (0-8), enable (0-1)");
      Serial.println("Type 'back' to return to main menu.");

      while(true) {
            if (Serial.available()) {
                  String cmd = Serial.readStringUntil('\n'); cmd.trim(); cmd.toLowerCase();
                  if (cmd == "back") { printMainMenu(); return; }
                  
                  if (cmd.startsWith("set ")) {
                  int spaceIdx = cmd.lastIndexOf(' ');
                  String param = cmd.substring(4, spaceIdx);
                  int val = cmd.substring(spaceIdx + 1).toInt();

                  if (param == "odr") { mmc_cfg.outputRate = val; Serial.printf("MMC ODR set to %d\n", val); }
                  else if (param == "bw") { mmc_cfg.bandwidth = val; Serial.printf("MMC BW set to %d\n", val); }
                  else if (param == "setfreq") { mmc_cfg.setFrequency = val; Serial.printf("MMC Set Frequency set to %d\n", val); }
                  else if (param == "enable") { mmc_cfg.chip_enable = val; Serial.printf("MMC Enable set to %d\n", val); }
                  else Serial.println("Unknown parameter.");
                  }
            }
            vTaskDelay(10);
      }
}

void SixthSense_Config::fusionMenu(Fusion_Config_t &cfg) {
      Serial.println("\n--- FUSION SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set gain 0.3");
      Serial.println("Params (Int): enable (0-1), axis (1-3)");
      Serial.println("Params (Float): gain, accel_rej, mag_rej, recovery, gyro_thresh, gyro_period");
      Serial.println("Type 'back' to return to main menu.");

      while(true) {
            if (Serial.available()) {
                  String cmd = Serial.readStringUntil('\n'); cmd.trim(); cmd.toLowerCase();
                  if (cmd == "back") { printMainMenu(); return; }
                  
                  if (cmd.startsWith("set ")) {
                  int spaceIdx = cmd.lastIndexOf(' ');
                  String param = cmd.substring(4, spaceIdx);
                  float val = cmd.substring(spaceIdx + 1).toFloat();

                  if (param == "enable") { cfg.fusion_enable = (bool)val; Serial.printf("Fusion Enable set to %d\n", (int)val); }
                  else if (param == "axis") { cfg.axis_setting = (int)val; Serial.printf("Axis Setting set to %d\n", (int)val); }
                  else if (param == "gain") { cfg.fusion_gain = val; Serial.printf("Gain set to %.3f\n", val); }
                  else if (param == "accel_rej") { cfg.accel_rejection = val; Serial.printf("Accel Rejection set to %.2f\n", val); }
                  else if (param == "mag_rej") { cfg.mag_rejection = val; Serial.printf("Mag Rejection set to %.2f\n", val); }
                  else if (param == "recovery") { cfg.recovery_period = val; Serial.printf("Recovery Period set to %.2f\n", val); }
                  else if (param == "gyro_thresh") { cfg.gyro_stationary_threshold = val; Serial.printf("Gyro Thresh set to %.2f\n", val); }
                  else if (param == "gyro_period") { cfg.gyro_stationary_period = val; Serial.printf("Gyro Period set to %.2f\n", val); }
                  else Serial.println("Unknown parameter.");
                  }
            }
            vTaskDelay(10);
      }
}

// --------------------------------------------------------
// NVS MEMORY OPERATIONS
// --------------------------------------------------------

bool SixthSense_Config::loadConfigs(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg) {
      if (nvs_open("SixthSense", NVS_READONLY, &my_handle) != ESP_OK) return false;
      
      bool success = true;
      size_t len = sizeof(ICM_Config_t);
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

      // Grab the exposed calibration structs from the hardware wrapper
      ICM_Cal_t icmCal = imu->getICMCal();
      MMC_Cal_t mmcCal = imu->getMMCCal();

      nvs_set_blob(my_handle, "icm_cal", &icmCal, sizeof(ICM_Cal_t));
      nvs_set_blob(my_handle, "mmc_cal", &mmcCal, sizeof(MMC_Cal_t));
      
      nvs_commit(my_handle);
      nvs_close(my_handle);
      
      Serial.println("[Config] Calibration matrices saved to flash.");
      return true;
}

bool SixthSense_Config::loadCalibration() {
      if (nvs_open("SixthSense", NVS_READONLY, &my_handle) != ESP_OK) return false;

      ICM_Cal_t icmCal;
      MMC_Cal_t mmcCal;
      bool success = true;

      size_t len = sizeof(ICM_Cal_t);
      if (nvs_get_blob(my_handle, "icm_cal", &icmCal, &len) == ESP_OK) {
            imu->setICMCal(icmCal);
      } else {
            success = false;
      }
      
      len = sizeof(MMC_Cal_t);
      if (nvs_get_blob(my_handle, "mmc_cal", &mmcCal, &len) == ESP_OK) {
            imu->setMMCCal(mmcCal);
      } else {
            success = false;
      }
      
      nvs_close(my_handle);
      return success;
}