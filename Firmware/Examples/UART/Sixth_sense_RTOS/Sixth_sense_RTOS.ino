#include <Arduino.h>
#include "SixthSense_IMU.h"
#include "SixthSense_Config.h"
#include "SixthSense_IO.h"

// Global instances
SixthSense_IMU imu;
SixthSense_IO ioModule;
SixthSense_Config configManager(&imu, &ioModule);


void setup() {
      //Set led to on to indicate starting operation
      pinMode(PIN_STATUS_LED, OUTPUT);
      digitalWrite(PIN_STATUS_LED, LOW); // Default solid ON before transmission starts

      Serial.setTxBufferSize(4096);
      Serial.begin();
      Serial.setTxTimeoutMs(0); // Drops data instead of blocking when monitor is closed
      vTaskDelay(pdMS_TO_TICKS(200));//Do not block execution if no serial, just run sensor
      
      Serial.println("Initializing SixthSense System...");

      ICM_Config_t icm_cfg;
      MMC_Config_t mmc_cfg;
      Fusion_Config_t fusion_cfg;
      IO_Config_t io_cfg;
      Wireless_Config_t wireless_cfg;

      configManager.begin(icm_cfg, mmc_cfg, fusion_cfg, io_cfg, wireless_cfg);//Load in config stored in flash memory

      imu.sensor_init(icm_cfg, mmc_cfg, fusion_cfg);

      // Allow ESP32-S3 boot spike to settle into low steady-state current
      // Stagger by CAN ID so multiple peripherals don't burst simultaneously
      //vTaskDelay(pdMS_TO_TICKS(200 + (io_cfg.local_can_id % 100) * 50));

      ioModule.begin(io_cfg, wireless_cfg);

      Serial.println("Starting in 2 seconds:");

      // Link IO Mailbox to IMU
      imu.setOutputQueue(ioModule.getIMUDataQueue());

      vTaskDelay(pdMS_TO_TICKS(2000));//Give some time for user to read debug message

      digitalWrite(PIN_STATUS_LED, HIGH); //Turn LED off and yield io IO RTOS task to control LED operation

      // Start Tasks
      configManager.RTOS_startConfigTask(1, 2);
      ioModule.RTOS_startIOTasks(0, 4);        // Pinned to Core 0
      imu.RTOS_startReadingTask(1, 5);         // Pinned to Core 1
}

void loop() {
      vTaskDelay(portMAX_DELAY); // Sleeps forever
}