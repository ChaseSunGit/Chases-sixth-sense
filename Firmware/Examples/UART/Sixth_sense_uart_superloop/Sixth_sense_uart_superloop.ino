#include <Arduino.h>
#include "SixthSense_IMU.h"
#include "SixthSense_Config.h"

// Global instances
SixthSense_IMU imu;
SixthSense_Config configManager(&imu);

void setup() {
      Serial.setTxBufferSize(4096);
      Serial.begin();
      while (!Serial) { delay(10); }
      

      Serial.println("Initializing SixthSense System...");

      //Initialize config structs
      ICM_Config_t icm_cfg;
      MMC_Config_t mmc_cfg;
      Fusion_Config_t fusion_cfg;

      // Initialize configuration manager and load saved configurations from NVS
      configManager.begin(icm_cfg, mmc_cfg, fusion_cfg);

      // Initialize the IMU with the loaded configurations
      if (!imu.sensor_init(icm_cfg, mmc_cfg, fusion_cfg)) {
            Serial.println("Warning: IMU initialization returned false (sensors may be disabled or missing).");
      }

      // Start FreeRTOS tasks for background reading and UART CLI monitoring
      configManager.RTOS_startConfigTask(1, 2);
      vTaskDelay(pdMS_TO_TICKS(5000));
      imu.RTOS_startReadingTask(1, 5);
      

      Serial.println("System Ready. Type 'config' to enter the configuration menu.");
}

void loop() {

      vTaskDelay(portMAX_DELAY); // Sleeps forever, 0% CPU usage
      // Sensor reading and configuration are handled in RTOS tasks.
      // We can print the data here periodically if fusion is active.
      //Fusion_Data_t data = imu.getFusionData();
      
      //Serial.printf("Roll: %.2f | Pitch: %.2f | Yaw: %.2f\n", data.roll, data.pitch, data.yaw);
                        
      //vTaskDelay(pdMS_TO_TICKS(100)); // Print at 10Hz
}