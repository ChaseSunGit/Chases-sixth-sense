/**
 * \file SixthSense_Config.cpp
 * \brief Implementation of the configuration manager with sub-menus.
 *
 * \author    Chase Sun
 */

#include "SixthSense_Config.h"

SixthSense_Config::SixthSense_Config(SixthSense_IMU* imu_ptr, SixthSense_IO* io_ptr) : imu(imu_ptr), io(io_ptr) {}

bool SixthSense_Config::begin(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg, IO_Config_t &io_cfg, Wireless_Config_t &w_cfg) {
      // Initialize NVS
      esp_err_t err = nvs_flash_init();

      if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());//Wipe all pages to start anew
            err = nvs_flash_init();
      }
      ESP_ERROR_CHECK(err);

      // Attempt to load. If either fails, it's the first time running (or memory was wiped)
      bool configsLoaded = loadConfigs(icm_cfg, mmc_cfg, fusion_cfg, io_cfg, w_cfg);
      // There is a fundamental difference between these functions 
      //- loadConfigs grabs stored config values in memory and loads the local config structs (but not on the sensor)
      //- loadCalibration loads stored config constants directly into the sensors during runtime, no need to reinitialize sensors
      bool calsLoaded = loadCalibration();

      if (!configsLoaded || !calsLoaded) {
            Serial.println("[NVS] No saved configurations found. Performing initial setup...");
            factoryReset(icm_cfg, mmc_cfg, fusion_cfg, io_cfg, w_cfg);
            saveConfigs(icm_cfg, mmc_cfg, fusion_cfg, io_cfg, w_cfg);
            saveCalibration();
      } else {
            Serial.println("[NVS] Configurations and calibrations successfully loaded from flash.");
      }

      // Check mode to change clock frequency
      // Central mode - 240mhz, high speed requirement for handling peripherals
      // Peripheral mode - 80mhz, slow speed to save battery power
      if (io_cfg.mode == NODE_CENTRAL){
            setCpuFrequencyMhz(240); 
            Serial.printf("[Config] Central mode, CPU Freq: %u MHz\n", getCpuFrequencyMhz());
      } else {
            setCpuFrequencyMhz(80); 
            Serial.printf("[Config] Peripheral mode, CPU Freq: %u MHz\n", getCpuFrequencyMhz());
      }
      
      return true;
}

void SixthSense_Config::checkUART() {
      static char rxBuffer[64];
      static uint8_t rxIndex = 0;

      while (Serial.available()) {
            char c = (char)Serial.read();

            if (c == '\n' || c == '\r') {
                  if (rxIndex > 0) {
                        rxBuffer[rxIndex] = '\0'; // Null terminate
                        
                        if (strcasecmp(rxBuffer, "config") == 0) {
                              rxIndex = 0;
                              enterConfigMode();
                              return;
                        }

                        // Diagnostic: print what actually arrived before Enter
                        Serial.printf("[UART-DBG] Rejected (%u bytes): ", rxIndex);
                        for (uint8_t i = 0; i < rxIndex; i++) {
                              Serial.printf("0x%02X ", (uint8_t)rxBuffer[i]);
                        }
                        Serial.println();

                        rxIndex = 0; // Discard unrecognized command
                  }
            } else {
                  if (rxIndex < 63) {
                        rxBuffer[rxIndex++] = c;
                  } else {
                        Serial.println("[UART-DBG] Buffer overflow (>63 bytes), resetting.");
                        rxIndex = 0;
                  }
            }
      }
}

void SixthSense_Config::factoryReset(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg, IO_Config_t &io_cfg, Wireless_Config_t &w_cfg) {
      Serial.println("[Config] Generating default configurations...");
      SixthSense_IMU::returnDefaultConfig(icm_cfg, mmc_cfg, fusion_cfg);
      SixthSense_IO::returnDefaultConfig(io_cfg);
      SixthSense_Wireless::returnDefaultConfig(w_cfg);
      
      // Zero out IMU calibrations (using Identity Matrix for Magnetometer Soft Iron)
      ICM_Cal_t icm_cal = { {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f} }; // all zeros in offsets
      MMC_Cal_t mmc_cal = { 
            {0.0f, 0.0f, 0.0f}, 
            { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} } // zero out offset, set softiron matrix to I
      };

      imu->setICMCal(icm_cal);
      imu->setMMCCal(mmc_cal);
}

void SixthSense_Config::enterConfigMode() {
      imu->RTOS_suspendReading(); // Halt background sensor tasks

      // Halt IO immediately to disable all wireless transmission and CAN bus activity during config mode
      if (io != nullptr) {
            io->RTOS_stopIOTasks();
      }

      //Set status led to on
      pinMode(PIN_STATUS_LED, OUTPUT);
      digitalWrite(PIN_STATUS_LED, LOW);

      Serial.printf("\n\n\n\n\n\n\n\n\n\n");
      Serial.println("=== ENTERING CONFIGURATION MODE ===");
      
      ICM_Config_t icm_cfg;
      MMC_Config_t mmc_cfg;
      Fusion_Config_t fusion_cfg;
      IO_Config_t io_cfg;
      Wireless_Config_t w_cfg;
      
      loadConfigs(icm_cfg, mmc_cfg, fusion_cfg, io_cfg, w_cfg); // Load the configs from memory

      bool icm_currently_en = icm_cfg.chip_enable;
      bool mmc_currently_en = mmc_cfg.chip_enable;
      //Store these freshly loaded values to track if sensors are actually enabled before allowing calibration

      printMainMenu();

      bool inConfig = true;
      String cmdBuffer = "";

      while (inConfig) {
            while (Serial.available()) {
                  char c = (char)Serial.read();

                  if (c == '\r' || c == '\n') {
                        cmdBuffer.trim();
                        cmdBuffer.toLowerCase();

                        if (cmdBuffer.length() > 0) {
                              if (cmdBuffer == "exit_save") {
                                    Serial.println("Exiting... Saving and reinitializing sensor & wireless...");
                                    saveConfigs(icm_cfg, mmc_cfg, fusion_cfg, io_cfg, w_cfg);
                                    saveCalibration();

                                    // Check mode to change clock frequency
                                    // Central mode - 240mhz, high speed requirement for handling peripherals
                                    // Peripheral mode - 80mhz, slow speed to save battery power
                                    if (io_cfg.mode == NODE_CENTRAL){
                                          setCpuFrequencyMhz(240); 
                                          Serial.printf("[Config] Central mode, CPU Freq: %u MHz\n", getCpuFrequencyMhz());
                                    } else {
                                          setCpuFrequencyMhz(80); 
                                          Serial.printf("[Config] Peripheral mode, CPU Freq: %u MHz\n", getCpuFrequencyMhz());
                                    }

                                    imu->sensor_init(icm_cfg, mmc_cfg, fusion_cfg);
                                    if (io != nullptr) {
                                          io->begin(io_cfg, w_cfg);       // Reconfigures filters, node mode, queues, and restarts ESP-NOW / TWAI
                                          io->RTOS_startIOTasks();      // Re-launches FreeRTOS TX/RX tasks with updated configuration
                                    }
                                    inConfig = false;
                              } else if (cmdBuffer == "exit_no_save") {
                                    Serial.println("Exiting... Config changes discarded, reverting from memory...");
                                    loadCalibration(); // Revert any in-RAM calibration changes back to saved state
                                    loadConfigs(icm_cfg, mmc_cfg, fusion_cfg, io_cfg, w_cfg); // Revert unsaved settings
                                    if (io != nullptr) {
                                          io->begin(io_cfg, w_cfg);       
                                          io->RTOS_startIOTasks();      
                                    }
                                    inConfig = false;
                              } else if (cmdBuffer == "menu") {
                                    printMainMenu();
                              } else if (cmdBuffer == "accel_gyro") {
                                    icmMenu(icm_cfg);
                              } else if (cmdBuffer == "mag") {
                                    magMenu(mmc_cfg);
                              } else if (cmdBuffer == "fusion") {
                                    fusionMenu(fusion_cfg);
                              } else if (cmdBuffer == "io") {
                                    ioMenu(io_cfg);
                              } else if (cmdBuffer == "wireless") {
                                    wirelessMenu(w_cfg);
                              } else if (cmdBuffer == "cal_accel") {
                                    if (icm_currently_en){
                                          Serial.println("Calibrating accelerometer - lay sensor flat...");
                                          for (int i = 2; i > 0; i--) {
                                                Serial.printf("Starting in %d...\n", i);
                                                vTaskDelay(pdMS_TO_TICKS(1000));
                                          }
                                          while (Serial.available()) Serial.read(); // Flush stray keystrokes
                                          imu->calibrateAccel(); 
                                          Serial.println("Calibration completed. Use 'exit_save' to commit to memory.");
                                    }
                                    else{
                                          Serial.println("Accelerometer disabled. Set ICM to enable and save and exit before calibrating.");
                                    }                                    
                              } else if (cmdBuffer == "cal_gyro") {
                                    if (icm_currently_en){
                                          Serial.println("Calibrating gyroscope - lay sensor flat and still...");
                                          for (int i = 2; i > 0; i--) {
                                                Serial.printf("Starting in %d...\n", i);
                                                vTaskDelay(pdMS_TO_TICKS(1000));
                                          }
                                          while (Serial.available()) Serial.read(); // Flush stray keystrokes
                                          imu->calibrateGyro(); 
                                          Serial.println("Calibration completed. Use 'exit_save' to commit to memory.");
                                    }
                                    else{
                                          Serial.println("Gyroscope disabled. Set ICM to enable and save and exit before calibrating.");
                                    }
                              } else if (cmdBuffer == "cal_mag") {
                                    if (mmc_currently_en){
                                          Serial.println("Calibrating magnetometer - move sensor in figure 8 fashion...");
                                          imu->calibrateMag(); 
                                          Serial.println("Calibration completed. Use 'exit_save' to commit to memory.");
                                    }
                                    else{
                                          Serial.println("Magnetometer disabled. Set MMC to enable and save and exit before calibrating.");
                                    }
                                    
                              } else if (cmdBuffer == "display_cal") {
                                    displayCalibration();
                              } else if (cmdBuffer == "factory_reset") {
                                    factoryReset(icm_cfg, mmc_cfg, fusion_cfg, io_cfg, w_cfg);
                                    Serial.println("Reset staged. Changes won't be applied until save and exit.");
                              } else {
                                    Serial.println("Unknown command. Type 'menu' for options.");
                              }

                              cmdBuffer = ""; // Reset buffer after processing command
                              if (!inConfig) break;
                        }
                  } else {
                        cmdBuffer += c;
                        if (cmdBuffer.length() > 64) {
                              cmdBuffer = ""; // Buffer overflow protection
                        }
                  }
            }
            vTaskDelay(pdMS_TO_TICKS(10)); 
      }

      Serial.println("=== RESUMING SENSOR OPERATIONS ===");
      vTaskDelay(pdMS_TO_TICKS(1000)); 
      digitalWrite(PIN_STATUS_LED, HIGH); // reset status led to off before returning to operation
      imu->RTOS_resumeReading();
}

void SixthSense_Config::printMainMenu() {
      Serial.printf("\n\n\n\n\n\n\n\n\n\n");
      Serial.println("--- MAIN MENU ---");
      Serial.println("Type a category to enter its sub-menu:");
      Serial.println("  accel_gyro    - Accelerometer + Gyroscope Settings (ODR, Range, BW, Enable)");
      Serial.println("  mag           - Magnetometer Settings (ODR, BW, Set Freq, Enable)");
      Serial.println("  fusion        - Sensor Fusion Settings (Gains, Rejections, Axis)");
      Serial.println("  io            - IO & Communications Settings (Mode, CAN ID, write content)");
      Serial.println("  wireless      - ESP-NOW Wireless Settings (Channel, Power, Phy Rate)");
      Serial.println("\nDirect Commands:");
      Serial.println("  cal_accel     - Calibrate Accelerometer");
      Serial.println("  cal_gyro      - Calibrate Gyroscope");
      Serial.println("  cal_mag       - Calibrate Magnetometer");
      Serial.println("  display_cal   - Print all calibration values");
      Serial.println("  factory_reset - Wipe flash and load defaults");
      Serial.println("  exit_save     - Save all changes, apply, and resume operation");
      Serial.println("  exit_no_save  - Discard all config changes but keep calibration, resume operation");
}

// --------------------------------------------------------
// SUB-MENUS
// --------------------------------------------------------
void SixthSense_Config::icmMenu(ICM_Config_t &icm_cfg) {
      Serial.printf("\n\n\n\n\n\n\n\n\n\n");
      Serial.println("--- ACCEL + GYRO SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set odr 5");
      Serial.println("Commands: disp_current_settings, back");
      Serial.println("Params:");
      Serial.println("  odr <1-8>");
      Serial.println("        Output data rate: 1 (12.5Hz), 2 (25Hz), 3 (50Hz), 4 (100Hz), 5 (200Hz, default), 6 (400Hz), 7 (800Hz), 8 (1600Hz)");
      Serial.println("  a_range <1-4>");
      Serial.println("        Accelerometer full scale range: 1 (±2g), 2 (±4g), 3 (±8g, default), 4 (±16g)");
      Serial.println("  a_bw <1-8>");
      Serial.println("        Accelerometer low pass filter bandwidth: 1 (16Hz), 2 (25Hz), 3 (34Hz), 4 (53Hz), 5 (73Hz, default), 6 (121Hz), 7 (180Hz), 8 (Bypassed)");
      Serial.println("  g_range <1-4>");
      Serial.println("        Gyroscope full scale range: 1 (±250dps), 2 (±500dps), 3 (±1000dps, default), 4 (±2000dps)");
      Serial.println("  g_bw <1-8>");
      Serial.println("        Gyroscope low pass filter bandwidth: 1 (16Hz), 2 (25Hz), 3 (34Hz), 4 (53Hz), 5 (73Hz, default), 6 (121Hz), 7 (180Hz), 8 (Bypassed)");
      Serial.println("  enable <0-1>");
      Serial.println("        ICM42607 chip enable: 0 - disable, 1 - enable");

      String cmdBuffer = "";

      while(true) {
            while (Serial.available()) {
                  char c = (char)Serial.read();

                  if (c == '\r' || c == '\n') {
                        cmdBuffer.trim();
                        cmdBuffer.toLowerCase();

                        if (cmdBuffer.length() > 0) {
                              if (cmdBuffer == "back") { 
                                    printMainMenu(); 
                                    return; 
                              } else if (cmdBuffer == "disp_current_settings") {
                                    Serial.println("\nReading active configuration directly from ICM-42607 registers...");
                                    if (!imu->readICMconfig()) {
                                          Serial.println("[ERROR] Failed to query ICM registers over SPI.");
                                    }
                              } else if (cmdBuffer.startsWith("set ")) {
                                    int spaceIdx = cmdBuffer.lastIndexOf(' ');
                                    if (spaceIdx <= 4) {
                                          Serial.println("Invalid format. Use: set <param> <value>");
                                    } else {
                                          String param = cmdBuffer.substring(4, spaceIdx);
                                          int val = cmdBuffer.substring(spaceIdx + 1).toInt();

                                          if (param == "odr") { 
                                                if (val < 1 || val > 8) {
                                                      Serial.println("Value out of range (1-8).");
                                                } else {
                                                      icm_cfg.outputRate = val; 
                                                      Serial.printf("Staged ICM ODR set to %.1f Hz\n", ICM42607::odr_table[val - 1]);
                                                }
                                          } else if (param == "a_range") { 
                                                if (val < 1 || val > 4) {
                                                      Serial.println("Value out of range (1-4).");
                                                } else {
                                                      icm_cfg.accel_range = val; 
                                                      Serial.printf("Staged Accel Range set to +/-%d g\n", ICM42607::accel_range_table[val - 1]);
                                                }
                                          } else if (param == "a_bw") { 
                                                if (val < 1 || val > 8) {
                                                      Serial.println("Value out of range (1-8).");
                                                } else {
                                                      icm_cfg.accel_bw = val; 
                                                      int bw = ICM42607::bw_table[val - 1];
                                                      if (bw == 0) Serial.println("Staged Accel Filter Bypassed");
                                                      else Serial.printf("Staged Accel Filter Bandwidth set to: %d Hz\n", bw);
                                                }
                                          } else if (param == "g_range") { 
                                                if (val < 1 || val > 4) {
                                                      Serial.println("Value out of range (1-4).");
                                                } else {
                                                      icm_cfg.gyro_range = val; 
                                                      Serial.printf("Staged Gyro Range set to +/-%d dps\n", ICM42607::gyro_range_table[val - 1]);
                                                }
                                          } else if (param == "g_bw") { 
                                                if (val < 1 || val > 8) {
                                                      Serial.println("Value out of range (1-8).");
                                                } else {
                                                      icm_cfg.gyro_bw = val; 
                                                      int bw = ICM42607::bw_table[val - 1];
                                                      if (bw == 0) Serial.println("Staged Gyro Filter Bypassed");
                                                      else Serial.printf("Staged Gyro Filter Bandwidth set to: %d Hz\n", bw);
                                                }
                                          } else if (param == "enable") { 
                                                if (val < 0 || val > 1) {
                                                      Serial.println("Value out of range (0-1).");
                                                } else {
                                                      icm_cfg.chip_enable = (val != 0); 
                                                      Serial.printf("Staged ICM Enable set to %d\n", val != 0); 
                                                }
                                          } else {
                                                Serial.println("Unknown parameter.");
                                          }
                                    }
                              } else {
                                    Serial.println("Unknown command. Type 'disp_current_settings', 'set <param> <val>', or 'back'.");
                              }
                              cmdBuffer = ""; // Reset accumulator after processing command
                        }
                  } else {
                        cmdBuffer += c;
                        if (cmdBuffer.length() > 64) {
                              cmdBuffer = "";
                        }
                  }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
      }
}

void SixthSense_Config::magMenu(MMC_Config_t &mmc_cfg) {
      Serial.printf("\n\n\n\n\n\n\n\n\n\n");
      Serial.println("--- MAGNETOMETER SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set odr 6");
      Serial.println("Commands: disp_current_settings, back");
      Serial.println("Params:");
      Serial.println("  odr <1-7>");
      Serial.println("        Continuous measurement rate: 1 (1Hz), 2 (10Hz), 3 (20Hz), 4 (50Hz), 5 (100Hz), 6 (200Hz, default), 7 (1000Hz)");
      Serial.println("  bw <1-4>");
      Serial.println("        Measurement bandwidth / filter duration: 1 (100Hz/8.0ms, default), 2 (200Hz/4.0ms), 3 (400Hz/2.0ms), 4 (800Hz/0.5ms)");
      Serial.println("  setfreq <0-8>");
      Serial.println("        SET/RESET auto-demagnetization interval: 0 (Disabled), 1 (1 sample), 2 (25 samples), 3 (75 samples), 4 (100 samples), 5 (250 samples), 6 (500 samples), 7 (1000 samples), 8 (2000 samples, default)");
      Serial.println("  enable <0-1>");
      Serial.println("        MMC5983MA chip enable: 0 - disable, 1 - enable");

      String cmdBuffer = "";

      while(true) {
            while (Serial.available()) {
                  char c = (char)Serial.read();

                  if (c == '\r' || c == '\n') {
                        cmdBuffer.trim();
                        cmdBuffer.toLowerCase();

                        if (cmdBuffer.length() > 0) {
                              if (cmdBuffer == "back") { 
                                    printMainMenu(); 
                                    return; 
                              } else if (cmdBuffer == "disp_current_settings") {
                                    Serial.println("\nReading active configuration directly from MMC5983MA registers...");
                                    MMC5983MA::parse_config(mmc_cfg);
                              } else if (cmdBuffer.startsWith("set ")) {
                                    int spaceIdx = cmdBuffer.lastIndexOf(' ');
                                    if (spaceIdx <= 4) {
                                          Serial.println("Invalid format. Use: set <param> <value>");
                                    } else {
                                          String param = cmdBuffer.substring(4, spaceIdx);
                                          int val = cmdBuffer.substring(spaceIdx + 1).toInt();

                                          if (param == "odr") {
                                                if (val < 1 || val > 7) {
                                                      Serial.println("Value out of range (1-7).");
                                                } else {
                                                      mmc_cfg.outputRate = val; 
                                                      Serial.printf("Staged MMC ODR set to index %d\n", val);
                                                }
                                          } else if (param == "bw") {
                                                if (val < 1 || val > 4) {
                                                      Serial.println("Value out of range (1-4).");
                                                } else {
                                                      // Rate 7 (1000 Hz) requires measurement time <= 0.5 ms -> BW setting 4 (800 Hz)
                                                      if (mmc_cfg.outputRate == 7 && val != 4) {
                                                            Serial.printf("[MMC-CONFIG-ERROR] 1000Hz (rate 7) requires bandwidth 4 (800Hz / 0.5ms)\n");
                                                      }
                                                      // Rate 6 (200 Hz) requires measurement time <= 4 ms -> BW setting >= 2 (200 Hz / 4ms)
                                                      else if (mmc_cfg.outputRate == 6 && val < 2) {
                                                            Serial.printf("[MMC-CONFIG-ERROR] 200Hz (rate 6) requires bandwidth >= 2 (200Hz / 4ms)\n");

                                                      } else{
                                                            mmc_cfg.bandwidth = val; 
                                                            Serial.printf("Staged MMC Bandwidth set to index %d\n", val);
                                                      }
                                                }
                                          } else if (param == "setfreq") {
                                                if (val < 0 || val > 8) {
                                                      Serial.println("Value out of range (0-8).");
                                                } else {
                                                      mmc_cfg.setFrequency = val; 
                                                      Serial.printf("Staged MMC Set Frequency set to index %d\n", val);
                                                }
                                          } else if (param == "enable") {
                                                if (val < 0 || val > 1) {
                                                      Serial.println("Value out of range (0-1).");
                                                } else {
                                                      mmc_cfg.chip_enable = (val != 0); 
                                                      Serial.printf("Staged MMC Enable set to %d\n", val != 0);
                                                }
                                          } else {
                                                Serial.println("Unknown parameter.");
                                          }
                                    }
                              } else {
                                    Serial.println("Unknown command. Type 'disp_current_settings', 'set <param> <val>', or 'back'.");
                              }
                              cmdBuffer = ""; // Reset accumulator after processing command
                        }
                  } else {
                        cmdBuffer += c;
                        if (cmdBuffer.length() > 64) {
                              cmdBuffer = "";
                        }
                  }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
      }
}

void SixthSense_Config::fusionMenu(Fusion_Config_t &fusion_cfg) {
      Serial.printf("\n\n\n\n\n\n\n\n\n\n");
      Serial.println("--- SENSOR FUSION SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set gain 0.2");
      Serial.println("Commands: disp_staged, back");
      Serial.println("Params (Integer):");
      Serial.println("  enable <0-1>");
      Serial.println("        AHRS algorithm enable: 0 - disable, 1 - enable");
      Serial.println("  axis <1-3>");
      Serial.println("        Output coordinate frame alignment: 1 (Standard/Flat), 2 (Microstrain standard, default), 3 (Microstrain upright)");
      Serial.println("Params (Float):");
      Serial.println("  gain <0.0-1.0>");
      Serial.println("        Fusion filter weighting factor [0.0 to 1.0]: Lower biases towards gyro integration, higher towards accel/mag (default: 0.20)");
      Serial.println("  accel_rej <0.0-180.0>");
      Serial.println("        Linear acceleration rejection angle (degrees): Angular error threshold before rejecting accel vector (default: 20.0 deg)");
      Serial.println("  mag_rej <0.0-180.0>");
      Serial.println("        Magnetic distortion rejection angle (degrees): Angular error threshold before rejecting mag vector (default: 10.0 deg)");
      Serial.println("  recovery <0.0-60.0>");
      Serial.println("        Rejection recovery period (seconds): Timeout duration before recovering from anomalous measurements (default: 0.50 s)");
      Serial.println("  gyro_thresh <0.0-50.0>");
      Serial.println("        Gyro stationary motion threshold (deg/s): Angular velocity limit below which the sensor is flagged stationary (default: 3.00 dps)");
      Serial.println("  gyro_period <0.1-600.0>");
      Serial.println("        Gyro stationary detection window (seconds): Duration of stillness required to calibrate drift offset (default: 3.00 s)");

      String cmdBuffer = "";

      while(true) {
            while (Serial.available()) {
                  char c = (char)Serial.read();

                  if (c == '\r' || c == '\n') {
                        cmdBuffer.trim();
                        cmdBuffer.toLowerCase();

                        if (cmdBuffer.length() > 0) {
                              if (cmdBuffer == "back") { 
                                    printMainMenu(); 
                                    return; 
                              } else if (cmdBuffer == "disp_staged") {
                                    Serial.println();
                                    Serial.println("--- Staged Fusion Configuration ---");
                                    Serial.printf("  Enable         : %d\n", fusion_cfg.fusion_enable);
                                    Serial.printf("  Axis Setting   : %u\n", fusion_cfg.axis_setting);
                                    Serial.printf("  Gain           : %.3f\n", fusion_cfg.fusion_gain);
                                    Serial.printf("  Accel Rejection: %.2f deg\n", fusion_cfg.accel_rejection);
                                    Serial.printf("  Mag Rejection  : %.2f deg\n", fusion_cfg.mag_rejection);
                                    Serial.printf("  Recovery Period: %.2f s\n", fusion_cfg.recovery_period);
                                    Serial.printf("  Gyro Threshold : %.2f dps\n", fusion_cfg.gyro_stationary_threshold);
                                    Serial.printf("  Gyro Period    : %.2f s\n", fusion_cfg.gyro_stationary_period);
                              } else if (cmdBuffer.startsWith("set ")) {
                                    int spaceIdx = cmdBuffer.lastIndexOf(' ');
                                    if (spaceIdx <= 4) {
                                          Serial.println("Invalid format. Use: set <param> <value>");
                                    } else {
                                          String param = cmdBuffer.substring(4, spaceIdx);
                                          float val = cmdBuffer.substring(spaceIdx + 1).toFloat();

                                          if (param == "enable") {
                                                int ival = (int)val;
                                                if (ival < 0 || ival > 1) {
                                                      Serial.println("Value out of range (0-1).");
                                                } else {
                                                      fusion_cfg.fusion_enable = (ival != 0);
                                                      Serial.printf("Staged Fusion Enable set to %d\n", fusion_cfg.fusion_enable);
                                                }
                                          } else if (param == "axis") {
                                                int ival = (int)val;
                                                if (ival < 1 || ival > 3) {
                                                      Serial.println("Value out of range (1-3).");
                                                } else {
                                                      fusion_cfg.axis_setting = (uint8_t)ival;
                                                      Serial.printf("Staged Axis Setting set to %u\n", fusion_cfg.axis_setting);
                                                }
                                          } else if (param == "gain") {
                                                if (val < 0.0f || val > 1.0f) {
                                                      Serial.println("Value out of range [0.0, 1.0].");
                                                } else {
                                                      fusion_cfg.fusion_gain = val;
                                                      Serial.printf("Staged Gain set to %.3f\n", val);
                                                }
                                          } else if (param == "accel_rej") {
                                                if (val <= 0.0f || val > 180.0f) {
                                                      Serial.println("Value out of range (0.0, 180.0].");
                                                } else {
                                                      fusion_cfg.accel_rejection = val;
                                                      Serial.printf("Staged Accel Rejection set to %.2f deg\n", val);
                                                }
                                          } else if (param == "mag_rej") {
                                                if (val <= 0.0f || val > 180.0f) {
                                                      Serial.println("Value out of range (0.0, 180.0].");
                                                } else {
                                                      fusion_cfg.mag_rejection = val;
                                                      Serial.printf("Staged Mag Rejection set to %.2f deg\n", val);
                                                }
                                          } else if (param == "recovery") {
                                                if (val < 0.0f || val > 60.0f) {
                                                      Serial.println("Value out of range [0.0, 60.0].");
                                                } else {
                                                      fusion_cfg.recovery_period = val;
                                                      Serial.printf("Staged Recovery Period set to %.2f s\n", val);
                                                }
                                          } else if (param == "gyro_thresh") {
                                                if (val < 0.0f || val > 50.0f) {
                                                      Serial.println("Value out of range [0.0, 50.0].");
                                                } else {
                                                      fusion_cfg.gyro_stationary_threshold = val;
                                                      Serial.printf("Staged Gyro Threshold set to %.2f dps\n", val);
                                                }
                                          } else if (param == "gyro_period") {
                                                if (val <= 0.0f || val > 600.0f) {
                                                      Serial.println("Value out of range (0.0, 600.0].");
                                                } else {
                                                      fusion_cfg.gyro_stationary_period = val;
                                                      Serial.printf("Staged Gyro Period set to %.2f s\n", val);
                                                }
                                          } else {
                                                Serial.println("Unknown parameter.");
                                          }
                                    }
                              } else {
                                    Serial.println("Unknown command. Type 'disp_staged', 'set <param> <val>', or 'back'.");
                              }
                              cmdBuffer = ""; // Reset accumulator after processing command
                        }
                  } else {
                        cmdBuffer += c;
                        if (cmdBuffer.length() > 64) {
                              cmdBuffer = "";
                        }
                  }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
      }
}

void SixthSense_Config::ioMenu(IO_Config_t &io_cfg) {
      Serial.printf("\n\n\n\n\n\n\n\n\n\n");
      Serial.println("--- IO & COMMUNICATION SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set mode 0");
      Serial.println("Commands: disp_staged, back");
      Serial.println("Params:");
      Serial.println("  can_id <10-2040>     - Local CAN Base ID (must be a multiple of 10), 100 default");
      Serial.println("  num_p <0-9>          - Number of active peripheral nodes configured");
      Serial.println("  p_id <1-9> <10-2040> - Peripheral CAN Base ID (index 1 to 9, multiple of 10)");
      Serial.println("  p_timeout <0-~>      - Peripheral timeout before marked inactive. Set timeout to 0 for no timeout");
      Serial.println("  mode <0-1>           - Node mode: 0 (Central UART, default), 1 (Peripheral CAN / Wireless)");
      Serial.println("  en_can_1mhz <0-1>    - Enable CAN 1MHz operation: 0 (Disable, 500kHz), 1 (Enable, default)");
      Serial.println("  w_accgyro <0-1>      - Write Accel/Gyro: 0 (Disable), 1 (Enable, default)");
      Serial.println("  w_mag <0-1>          - Write Magnetometer: 0 (Disable, default), 1 (Enable)");
      Serial.println("  w_euler <0-1>        - Write Euler Angles: 0 (Disable, default), 1 (Enable)");
      Serial.println("  w_quat <0-1>         - Write Quaternions: 0 (Disable, default), 1 (Enable)");
      Serial.println("  verbose <0-1>        - Verbose UART output: 0 (Disable), 1 (Enable, default). Disable for binary output on central node, or no uart on peripherial.");
      Serial.println("  listener_mode <0-1>  - Central node listener mode: 0 (Off, default), 1 (On). Turn on to disable central node IMU output for pure peripherial output.");

      String cmdBuffer = "";

      while(true) {
            while (Serial.available()) {
                  char c = (char)Serial.read();

                  if (c == '\r' || c == '\n') {
                        cmdBuffer.trim();
                        cmdBuffer.toLowerCase();

                        if (cmdBuffer.length() > 0) {
                              if (cmdBuffer == "back") { 
                                    printMainMenu(); 
                                    return; 
                              } else if (cmdBuffer == "disp_staged") {
                                    Serial.println();
                                    Serial.println("--- Staged IO Configuration ---");
                                    Serial.printf("  CAN ID              : %u\n", io_cfg.local_can_id);
                                    Serial.printf("  Num Peripherals     : %u\n", io_cfg.num_peripherals);
                                    Serial.printf("  Peripheral timeout  : %u\n", io_cfg.peripheral_timeout);
                                    
                                    Serial.print("  Peripheral sensor ID: ");
                                    for (int i = 0; i < 9; i++) {
                                          Serial.print(io_cfg.peripheral_can_ids[i]);
                                          if (i < 8) Serial.print(", ");
                                    }
                                    Serial.println();

                                    Serial.printf("  Mode                : %d\n", io_cfg.mode);
                                    Serial.printf("  Enable CAN HS       : %d\n", io_cfg.CAN_1MHZ_en);
                                    Serial.printf("  Write Acc/Gyro      : %d\n", io_cfg.write_accelGyro);
                                    Serial.printf("  Write Mag           : %d\n", io_cfg.write_mag);
                                    Serial.printf("  Write Euler         : %d\n", io_cfg.write_euler);
                                    Serial.printf("  Write Quat          : %d\n", io_cfg.write_quat);
                                    Serial.printf("  Verbose UART        : %d\n", io_cfg.verbose_uart);
                                    Serial.printf("  Listener mode       : %d\n", io_cfg.listener_mode);
                              } else if (cmdBuffer.startsWith("set ")) {
                                    int spaceIdx = cmdBuffer.lastIndexOf(' ');
                                    if (spaceIdx <= 4) {
                                          Serial.println("Invalid format. Use: set <param> <value>");
                                    } else {
                                          String param = cmdBuffer.substring(4, spaceIdx);
                                          int val = cmdBuffer.substring(spaceIdx + 1).toInt();

                                          if (param == "can_id") {
                                                if (val < 10 || val > 2040 || val % 10 != 0) {
                                                      Serial.println("Value must be between 10-2040 and a multiple of 10.");
                                                } else {
                                                      io_cfg.local_can_id = (uint16_t)val;
                                                      Serial.printf("Staged CAN ID set to %u\n", io_cfg.local_can_id);
                                                }
                                          } else if (param == "num_p") {
                                                if (val < 0 || val > 9) {
                                                      Serial.println("Value out of range (0-9).");
                                                } else {
                                                      io_cfg.num_peripherals = (uint8_t)val;
                                                      Serial.printf("Staged Num Peripherals set to %u\n", io_cfg.num_peripherals);
                                                }
                                          } else if (param == "p_timeout") {
                                                if (val < 0 || val > 1000000) {
                                                      Serial.println("Value out of range (0-1000000)");
                                                } else {
                                                      io_cfg.peripheral_timeout = val;
                                                      Serial.printf("Peripheral timeout set to %d ms\n", io_cfg.peripheral_timeout);
                                                }
                                          } else if (param.startsWith("p_id")) {
                                                int idx = param.substring(4).toInt(); // Extract index (e.g., "p_id1" -> 1)
                                                if (idx < 1 || idx > 9) {
                                                      Serial.println("Invalid peripheral index. Use: p_id1 through p_id9");
                                                } else if (val < 10 || val > 2040 || val % 10 != 0) {
                                                      Serial.println("Value must be between 10-2040 and a multiple of 10.");
                                                } else {
                                                      io_cfg.peripheral_can_ids[idx - 1] = (uint16_t)val;
                                                      Serial.printf("Staged Peripheral %d CAN ID set to %u\n", idx, io_cfg.peripheral_can_ids[idx - 1]);
                                                }
                                          } else if (param == "mode") {
                                                if (val < 0 || val > 1) {
                                                      Serial.println("Value out of range (0-1).");
                                                } else {
                                                      io_cfg.mode = (NodeMode)val;
                                                      Serial.printf("Staged Node Mode set to %d\n", io_cfg.mode);
                                                }
                                          } else if (param == "en_can_1mhz") {
                                                io_cfg.CAN_1MHZ_en = (val !=0);
                                                Serial.printf("Staged 1Mhz CAN enable set to %d\n", io_cfg.CAN_1MHZ_en);
                                          } else if (param == "w_accgyro") {
                                                io_cfg.write_accelGyro = (val != 0);
                                                Serial.printf("Staged Accel/Gyro Write set to %d\n", io_cfg.write_accelGyro);
                                          } else if (param == "w_mag") {
                                                io_cfg.write_mag = (val != 0);
                                                Serial.printf("Staged Mag Write set to %d\n", io_cfg.write_mag);
                                          } else if (param == "w_euler") {
                                                io_cfg.write_euler = (val != 0);
                                                Serial.printf("Staged Euler Write set to %d\n", io_cfg.write_euler);
                                          } else if (param == "w_quat") {
                                                io_cfg.write_quat = (val != 0);
                                                Serial.printf("Staged Quat Write set to %d\n", io_cfg.write_quat);
                                          } else if (param == "verbose") {
                                                io_cfg.verbose_uart = (val != 0);
                                                Serial.printf("Staged Verbose UART set to %d\n", io_cfg.verbose_uart);
                                          } else if (param == "listener_mode") {
                                                io_cfg.listener_mode = (val != 0);
                                                Serial.printf("Staged listener mode set to %d\n", io_cfg.listener_mode);
                                          }                                     
                                          else {
                                                Serial.println("Unknown parameter.");
                                          }
                                    }
                              } else {
                                    Serial.println("Unknown command. Type 'disp_staged', 'set <param> <val>', or 'back'.");
                              }
                              cmdBuffer = "";
                        }
                  } else {
                        cmdBuffer += c;
                        if (cmdBuffer.length() > 64) cmdBuffer = "";
                  }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
      }
}

void SixthSense_Config::wirelessMenu(Wireless_Config_t &w_cfg) {
      Serial.printf("\n\n\n\n\n\n\n\n\n\n");
      Serial.println("--- WIRELESS (ESP-NOW) SETTINGS ---");
      Serial.println("Format: set <param> <value> | Example: set channel 6");
      Serial.println("Commands: disp_staged, back");
      Serial.println("Params:");
      Serial.println("  channel <1-11>      - Wi-Fi Channel (1, 6, 11 recommended)");
      Serial.println("  tx_power <8-84>     - TX Power in 0.25dBm steps (84 = +21dBm max, default 48 = +12dBm)");
      Serial.println("  phy_rate <int>      - Wi-Fi PHY Rate enum integer (Default: 9):");
      Serial.println("                                0  = 1M   (802.11b Long Preamble)");
      Serial.println("                                3  = 11M  (802.11b Long Preamble)");
      Serial.println("                                9  = 24M  (802.11g OFDM - Default)");
      Serial.println("                                11 = 54M  (802.11g OFDM)");
      Serial.println("                                19 = MCS7 (802.11n HT20 ~65-72.2 Mbps)");
      Serial.println("  lr_mode <0-1>       - Long Range Mode: 0 (24Mbps OFDM, default), 1 (LR mode / 6Mbps)");

      String cmdBuffer = "";

      while(true) {
            while (Serial.available()) {
                  char c = (char)Serial.read();

                  if (c == '\r' || c == '\n') {
                        cmdBuffer.trim();
                        cmdBuffer.toLowerCase();

                        if (cmdBuffer.length() > 0) {
                              if (cmdBuffer == "back") { 
                                    printMainMenu(); 
                                    return; 
                              } else if (cmdBuffer == "disp_staged") {
                                    Serial.println();
                                    Serial.println("--- Staged Wireless Configuration ---");
                                    Serial.printf("  Channel     : %u\n", w_cfg.wifi_channel);
                                    Serial.printf("  TX Power    : %d (%.2f dBm)\n", w_cfg.tx_power_quarter_dbm, w_cfg.tx_power_quarter_dbm * 0.25f);
                                    Serial.printf("  PHY Rate    : %d\n", (int)w_cfg.phy_rate);
                                    Serial.printf("  LR Mode     : %d\n", w_cfg.enable_long_range_mode);
                              } else if (cmdBuffer.startsWith("set ")) {
                                    int spaceIdx = cmdBuffer.lastIndexOf(' ');
                                    if (spaceIdx <= 4) {
                                          Serial.println("Invalid format. Use: set <param> <value>");
                                    } else {
                                          String param = cmdBuffer.substring(4, spaceIdx);
                                          int val = cmdBuffer.substring(spaceIdx + 1).toInt();

                                          if (param == "channel") {
                                                if (val < 1 || val > 14) {
                                                      Serial.println("Value must be between 1 and 14.");
                                                } else {
                                                      w_cfg.wifi_channel = (uint8_t)val;
                                                      Serial.printf("Staged Wi-Fi Channel set to %u\n", w_cfg.wifi_channel);
                                                }
                                          } else if (param == "tx_power") {
                                                if (val < 8 || val > 84) {
                                                      Serial.println("Value must be between 8 and 84.");
                                                } else {
                                                      w_cfg.tx_power_quarter_dbm = (int8_t)val;
                                                      Serial.printf("Staged TX Power set to %d (%.2f dBm)\n", w_cfg.tx_power_quarter_dbm, w_cfg.tx_power_quarter_dbm * 0.25f);
                                                }
                                          } else if (param == "phy_rate") {
                                                w_cfg.phy_rate = (wifi_phy_rate_t)val;
                                                Serial.printf("Staged PHY Rate set to %d\n", (int)w_cfg.phy_rate);
                                          } else if (param == "lr_mode") {
                                                if (val < 0 || val > 1) {
                                                      Serial.println("Value must be 0 or 1.");
                                                } else {
                                                      w_cfg.enable_long_range_mode = (val != 0);
                                                      w_cfg.phy_rate = w_cfg.enable_long_range_mode ? WIFI_PHY_RATE_6M : WIFI_PHY_RATE_24M;
                                                      Serial.printf("Staged LR Mode set to %d\n", w_cfg.enable_long_range_mode);
                                                }
                                          } else {
                                                Serial.println("Unknown parameter.");
                                          }
                                    }
                              } else {
                                    Serial.println("Unknown command. Type 'disp_staged', 'set <param> <val>', or 'back'.");
                              }
                              cmdBuffer = "";
                        }
                  } else {
                        cmdBuffer += c;
                        if (cmdBuffer.length() > 64) cmdBuffer = "";
                  }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
      }
}

// --------------------------------------------------------
// NVS MEMORY OPERATIONS
// --------------------------------------------------------

bool SixthSense_Config::loadConfigs(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg, IO_Config_t &io_cfg, Wireless_Config_t &w_cfg) {
      if (nvs_open("SixthSense", NVS_READONLY, &imu_config_handle) != ESP_OK) return false;
      
      bool success = true;
      size_t len = sizeof(ICM_Config_t);
      if (nvs_get_blob(imu_config_handle, "icm_cfg", &icm_cfg, &len) != ESP_OK) success = false;
      
      len = sizeof(MMC_Config_t);
      if (nvs_get_blob(imu_config_handle, "mmc_cfg", &mmc_cfg, &len) != ESP_OK) success = false;
      
      len = sizeof(Fusion_Config_t);
      if (nvs_get_blob(imu_config_handle, "fus_cfg", &fusion_cfg, &len) != ESP_OK) success = false;

      len = sizeof(IO_Config_t);
      if (nvs_get_blob(imu_config_handle, "io_cfg", &io_cfg, &len) != ESP_OK) success = false;

      len = sizeof(Wireless_Config_t);
      if (nvs_get_blob(imu_config_handle, "w_cfg", &w_cfg, &len) != ESP_OK) success = false;
      
      nvs_close(imu_config_handle);
      return success;
}

bool SixthSense_Config::saveConfigs(const ICM_Config_t &icm_cfg, const MMC_Config_t &mmc_cfg, const Fusion_Config_t &fusion_cfg, const IO_Config_t &io_cfg, const Wireless_Config_t &w_cfg) {
      if (nvs_open("SixthSense", NVS_READWRITE, &imu_config_handle) != ESP_OK) return false;
      
      nvs_set_blob(imu_config_handle, "icm_cfg", &icm_cfg, sizeof(ICM_Config_t));
      nvs_set_blob(imu_config_handle, "mmc_cfg", &mmc_cfg, sizeof(MMC_Config_t));
      nvs_set_blob(imu_config_handle, "fus_cfg", &fusion_cfg, sizeof(Fusion_Config_t));
      nvs_set_blob(imu_config_handle, "io_cfg", &io_cfg, sizeof(IO_Config_t));
      nvs_set_blob(imu_config_handle, "w_cfg", &w_cfg, sizeof(Wireless_Config_t));
      
      nvs_commit(imu_config_handle);
      nvs_close(imu_config_handle);
      return true;
}

bool SixthSense_Config::saveCalibration() {
      if (nvs_open("SixthSense", NVS_READWRITE, &imu_config_handle) != ESP_OK) return false;

      // Grab the exposed calibration structs from the hardware wrapper
      ICM_Cal_t icmCal = imu->getICMCal();
      MMC_Cal_t mmcCal = imu->getMMCCal();

      nvs_set_blob(imu_config_handle, "icm_cal", &icmCal, sizeof(ICM_Cal_t));
      nvs_set_blob(imu_config_handle, "mmc_cal", &mmcCal, sizeof(MMC_Cal_t));
      
      nvs_commit(imu_config_handle);
      nvs_close(imu_config_handle);
      
      Serial.println("[Config] Calibration matrices saved to flash.");
      return true;
}

bool SixthSense_Config::loadCalibration() {
      if (nvs_open("SixthSense", NVS_READONLY, &imu_config_handle) != ESP_OK) return false;

      ICM_Cal_t icmCal;
      MMC_Cal_t mmcCal;
      bool success = true;

      size_t len = sizeof(ICM_Cal_t);
      if (nvs_get_blob(imu_config_handle, "icm_cal", &icmCal, &len) == ESP_OK) {
            imu->setICMCal(icmCal);
      } else {
            success = false;
      }
      
      len = sizeof(MMC_Cal_t);
      if (nvs_get_blob(imu_config_handle, "mmc_cal", &mmcCal, &len) == ESP_OK) {
            imu->setMMCCal(mmcCal);
      } else {
            success = false;
      }
      
      nvs_close(imu_config_handle);
      return success;
}

void SixthSense_Config::displayCalibration(){
      ICM_Cal_t icmCal = imu->getICMCal();
      MMC_Cal_t mmcCal = imu->getMMCCal();

      Serial.println("\n================ CURRENT CALIBRATION MATRICES ================");

      // 1. ICM-42607 Accelerometer Offsets (in g)
      Serial.println("[ICM-42607] Accelerometer Zero-G Offsets (g):");
      Serial.printf("  X: %+.6f | Y: %+.6f | Z: %+.6f\n", 
                        icmCal.accel_offset[0], icmCal.accel_offset[1], icmCal.accel_offset[2]);

      // 2. ICM-42607 Gyroscope Offsets (in dps)
      Serial.println("\n[ICM-42607] Gyroscope Zero-Rate Bias (dps):");
      Serial.printf("  X: %+.6f | Y: %+.6f | Z: %+.6f\n", 
                        icmCal.gyro_offset[0], icmCal.gyro_offset[1], icmCal.gyro_offset[2]);

      // 3. MMC5983MA Magnetometer Hard-Iron Offsets (in Gauss)
      Serial.println("\n[MMC5983MA] Hard-Iron Centroid Offsets (Gauss):");
      Serial.printf("  X: %+.6f | Y: %+.6f | Z: %+.6f\n", 
                        mmcCal.offset[0], mmcCal.offset[1], mmcCal.offset[2]);

      // 4. MMC5983MA Magnetometer Soft-Iron Matrix W (3x3 Tensor)
      Serial.println("\n[MMC5983MA] Soft-Iron Correction Matrix (W):");
      for (int r = 0; r < 3; ++r) {
            Serial.printf("  [ %+.6f,  %+.6f,  %+.6f ]\n", 
                        mmcCal.w[r][0], mmcCal.w[r][1], mmcCal.w[r][2]);
      }

    Serial.println("=============================================================");
}

//RTOS block
bool SixthSense_Config::RTOS_startConfigTask(BaseType_t core_id, UBaseType_t priority) {
      if (config_task_handle != NULL) {
            return true; // Task is already running
      }

      BaseType_t result = xTaskCreatePinnedToCore(
            Config_Task_Trampoline, 
            "Config_UART_Task", 
            4096,                 // 4KB stack is sufficient for CLI parsing
            this,                 // Pass class instance pointer as parameter
            priority,             // Low priority (e.g., 1) so it doesn't starve the IMU task
            &config_task_handle, 
            core_id
      );

      return (result == pdPASS);
}

void SixthSense_Config::RTOS_stopConfigTask() {
      if (config_task_handle != NULL) {
            vTaskDelete(config_task_handle);
            config_task_handle = NULL;
      }
}

// Static trampoline forwarding execution back to member methods
void SixthSense_Config::Config_Task_Trampoline(void *pvParameters) {
      SixthSense_Config *cfg = static_cast<SixthSense_Config*>(pvParameters);

      while (1) {
            cfg->checkUART();
            vTaskDelay(pdMS_TO_TICKS(20)); // Yield 5ms between UART checks
      }
}