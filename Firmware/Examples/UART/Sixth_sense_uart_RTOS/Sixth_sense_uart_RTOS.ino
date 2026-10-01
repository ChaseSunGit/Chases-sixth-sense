#include <Arduino.h>
#include "ICM42607_Driver.h"
#include "MMC5983MA_Driver.h"
#include "Config.h"
#include "Sensor_Fusion.h"
#include "CAN_Comm.h"
#include "BLE_Comm.h"

// RTOS Task Handle
TaskHandle_t SensorTaskHandle = NULL;
bool ble_enabled = false;

// Main Sensor Processing Task
void SensorTask(void *pvParameters) {
      uint32_t last_time = micros();

      while (true) {
            // Block indefinitely until an ISR gives a notification (saves power)
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Check which data is ready based on the flags set in the drivers
            if (new_data_ready_ICM) {
                  new_data_ready_ICM = false;
                  ICM_single_read(); // Unpack DMA buffer

                  float dt = (micros() - last_time) / 1000000.0f;
                  last_time = micros();

                  // Perform Sensor Fusion
                  Update_Sensor_Fusion(
                  ICM_Data_Holder.ax, ICM_Data_Holder.ay, ICM_Data_Holder.az,
                  ICM_Data_Holder.gx, ICM_Data_Holder.gy, ICM_Data_Holder.gz,
                  MMC_Data_Holder.mx, MMC_Data_Holder.my, MMC_Data_Holder.mz,
                  dt
                  );

                  // Transmit data on CAN bus matching ICM rate
                  CAN_send_sensor_data(Fusion_Data.q, Fusion_Data.roll, Fusion_Data.pitch, Fusion_Data.yaw);

                  // Transmit over BLE if toggled on
                  if (ble_enabled) {
                  BLE_send_sensor_data(Fusion_Data.q, Fusion_Data.roll, Fusion_Data.pitch, Fusion_Data.yaw);
                  }
            }

            if (new_mag_data_ready) {
                  new_mag_data_ready = false;
                  MMC_single_read(); // Unpack Mag DMA buffer
            }
      }
}

void setup() {
      Serial.begin(115200);
      while (!Serial && millis() < 3000) { delay(10); }

      // Initialize Storage and load constants
      Storage_Init();
      Storage_Load_Calibration();

      // Initialize Communications
      CAN_init();
      BLE_init();

      // Initialize ICM and MMC[cite: 7]
      if (!ICM_init_chip(2)) Serial.println("ICM init failed!");
      if (!MMC_init_chip(2)) Serial.println("MMC init failed!");

      // Create RTOS Task (Priority 2, pinned to core 1)
      xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 2, &SensorTaskHandle, 1);

      // Attach Hardware Interrupts[cite: 7]
      pinMode(PIN_INT_ICM, INPUT);
      attachInterrupt(digitalPinToInterrupt(PIN_INT_ICM), IMU_ISR_dataReady, RISING);
      
      // Note: Assuming PIN_INT_MMC is defined in your environment
      // pinMode(PIN_INT_MMC, INPUT);
      // attachInterrupt(digitalPinToInterrupt(PIN_INT_MMC), MMC_ISR_dataReady, RISING);
}

void loop() {
      // Empty. FreeRTOS handles execution via SensorTask.
      vTaskDelete(NULL); 
}