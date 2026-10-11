#include <Arduino.h>
#include "SixthSense_IMU.h"
#include "SixthSense_Config.h"
#include "SixthSense_IO.h"

// Global instances
SixthSense_IMU imu;
SixthSense_IO ioModule;
SixthSense_Config configManager(&imu, &ioModule);


void setup() {
      Serial.setTxBufferSize(4096);
      Serial.begin();
      vTaskDelay(pdMS_TO_TICKS(200));//Do not block execution if no serial, just run sensor

      Serial.println("Initializing SixthSense System...");

      ICM_Config_t icm_cfg;
      MMC_Config_t mmc_cfg;
      Fusion_Config_t fusion_cfg;
      IO_Config_t io_cfg;

      configManager.begin(icm_cfg, mmc_cfg, fusion_cfg, io_cfg);

      if (!imu.sensor_init(icm_cfg, mmc_cfg, fusion_cfg)) {
            Serial.println("Warning: IMU initialization returned false.");
      }

      ioModule.begin(io_cfg);

      // Link IO Mailbox to IMU
      imu.setOutputQueue(ioModule.getIMUDataQueue());

      vTaskDelay(pdMS_TO_TICKS(5000));//Give some time for user to read debug message

      // Start Tasks
      configManager.RTOS_startConfigTask(1, 2);
      ioModule.RTOS_startIOTasks(0, 4);        // Pinned to Core 0
      imu.RTOS_startReadingTask(1, 5);         // Pinned to Core 1
}

void loop() {
      vTaskDelay(portMAX_DELAY); // Sleeps forever
}