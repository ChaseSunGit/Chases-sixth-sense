#include "ICM42607_Driver.h"

#define CONTROL_LOOP_FREQUENCY 200 //This is the frequency of the low level control loop and is actuated by back IMU interrupt as the master timer

void setup(void) {
      Serial.begin(115200);
      unsigned long start = millis();
      while (!Serial && (millis() - start < 3000)) {
            delay(10);
      }
      Serial.println("Beginning initialization process");


      // Initialize SPI hardware
      if (!ICM_SPI_config()) {
            Serial.println("SPI init failed!");
            return;
      }

      // Initialize ICM with all relevant settings
      if (!ICM_init_chip(2)) {
            Serial.println("ICM initialization failed!");
            return;
      }
      else{
            Serial.println("ICM initialization success!");
      }
      
      // Allocate DMA buffers and pre-arm descriptor
      if (!ICM_DMA_config()) {
            Serial.println("DMA buffer allocation failed!");
            return;
      }
      else{
            Serial.println("DMA buffer allocation success!");
      }

      pinMode(PIN_INT_ICM, INPUT);
      attachInterrupt(digitalPinToInterrupt(PIN_INT_ICM), IMU_ISR_dataReady, RISING);
      Serial.println("Interrupt set for ICM42607");
      Serial.println("Calibrating accelerometer");
      ICM_accel_calib(200);
      Serial.println("Calibrating gyroscope");
      ICM_gyro_calib(200);
      Serial.println("Ready to collect data:");
}

void loop() {
      // Check if new data has arrived from the DMA engine
      if (new_data_ready) {
            new_data_ready = false;

            // Drain the completed transaction from the SPI driver queue to free the hardware
            ICM_single_read();
            //Serial.printf("ICM Data: %.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n",ICM_Data_Holder.ax,ICM_Data_Holder.ay,ICM_Data_Holder.az,ICM_Data_Holder.gx,ICM_Data_Holder.gy,ICM_Data_Holder.gz);
            ICM_Kalman_fusion(0.005);
            Serial.printf("%.4f,%.4f\n",ICM_Data_Holder.roll,ICM_Data_Holder.pitch);
            

      }
}