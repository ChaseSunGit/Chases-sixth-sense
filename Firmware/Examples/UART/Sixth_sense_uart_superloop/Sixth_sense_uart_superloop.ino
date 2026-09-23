#include "ICM42607_Driver.h"

#define CONTROL_LOOP_FREQUENCY 200 //This is the frequency of the low level control loop and is actuated by back IMU interrupt as the master timer

void setup(void) {
      Serial.begin(112500);
      while (!Serial){
            delay(10); // Don't start until serial is ready
      }


      // Initialize SPI hardware
      if (!ICM_SPI_config()) {
            Serial.println("SPI init failed!");
            return;
      }

      // Initialize ICM with all relevant settings
      if (!ICM_init_chip(CONTROL_LOOP_FREQUENCY)) {
            Serial.println("ICM initialization failed!");
            return;
      }
      
      // Allocate DMA buffers and pre-arm descriptor
      if (!ICM_DMA_config()) {
            Serial.println("DMA buffer allocation failed!");
            return;
      }

      pinMode(PIN_INT_ICM, INPUT);
      attachInterrupt(digitalPinToInterrupt(PIN_INT_ICM), IMU_ISR_dataReady, RISING);
      Serial.println("Interrupt set for ICM42607");
}

void loop() {
      // Check if new data has arrived from the DMA engine
      if (new_data_ready) {
            new_data_ready = false;

            // Drain the completed transaction from the SPI driver queue to free the hardware
            ICM_single_read();
            Serial.printf("ICM Data: %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",ICM_Data_Holder.ax,ICM_Data_Holder.ay,ICM_Data_Holder.az,ICM_Data_Holder.gx,ICM_Data_Holder.gy,ICM_Data_Holder.gz);

      }
}