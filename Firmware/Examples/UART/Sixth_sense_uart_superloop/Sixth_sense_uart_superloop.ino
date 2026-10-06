#include "ICM42607_Driver.h"
#include "MMC5983MA_Driver.h"

TaskHandle_t SensorTaskHandle = NULL;
//Temporary boolean for RTOS vs superloop mode
bool using_RTOS = false;

void setup(void) {
      Serial.setTxBufferSize(1024); // Expand TX ring buffer to 1KB to avoid dropping serial buffers
      Serial.begin(921600);
      unsigned long start = millis();
      while (!Serial && (millis() - start < 3000)) {
            delay(10);
      }
      Serial.println("[General] Beginning initialization process");


      // Initialize ICM with all relevant settings
      
      if (!ICM_init_chip(2)) {
            Serial.println("[ICM] ICM initialization failed!");
            return;
      }
      else{
            Serial.println("[ICM] ICM initialization success!");
      }


      // Initialize ICM with all relevant settings
      
      if (!MMC_init_chip(6)) {
            Serial.println("[MMC] MMC initialization failed!");
            return;
      }
      else{
            Serial.println("[MMC] MMC initialization success!");
      }
      

      pinMode(PIN_INT_ICM, INPUT);
      attachInterrupt(digitalPinToInterrupt(PIN_INT_ICM), ICM_ISR_dataReady, RISING);
      
      int pinState = digitalRead(PIN_INT_MMC); 
      Serial.println("[ICM] Interrupt set for ICM42607");
      Serial.println("[ICM] Calibrating accelerometer");
      //ICM_accel_calib(200);
      Serial.println("[ICM] Calibrating gyroscope");
      //ICM_gyro_calib(200);
      Serial.println("[MMC] Calibrating magnetometer - move sensor in figure 8 pattern");
      Calibrate_Full_Soft_Iron();

      Serial.println("[General] Ready to collect data:");
      Serial.printf("[MMC] Mag calib offset: x %.4f\ty %.4f\tz %.4f\n",MMC_Data_Holder.offset[0], MMC_Data_Holder.offset[1], MMC_Data_Holder.offset[2]);
      Serial.printf("[MMC] Mag calib matrix:\n%.4f\t%.4f\t%.4f\n",MMC_Data_Holder.W[0][0], MMC_Data_Holder.W[0][1], MMC_Data_Holder.W[0][2]);
      Serial.printf("%.4f\t%.4f\t%.4f\n",MMC_Data_Holder.W[1][0], MMC_Data_Holder.W[1][1], MMC_Data_Holder.W[1][2]);
      Serial.printf("%.4f\t%.4f\t%.4f\n",MMC_Data_Holder.W[2][0], MMC_Data_Holder.W[2][1], MMC_Data_Holder.W[2][2]);
}

void loop() {
      // Check if new data has arrived from the DMA engine
      if (new_data_ready_ICM) {
            new_data_ready_ICM = false;

            // Drain the completed transaction from the SPI driver queue to free the hardware
            ICM_single_read();
            //Serial.printf("ICM Data: %.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n",ICM_Data_Holder.ax,ICM_Data_Holder.ay,ICM_Data_Holder.az,ICM_Data_Holder.gx,ICM_Data_Holder.gy,ICM_Data_Holder.gz);
            
            //Serial.printf("MAG pin state %d\n",digitalRead(PIN_INT_MMC));
            //Serial.printf("%.4f,%.4f\n",ICM_Data_Holder.roll,ICM_Data_Holder.pitch);
            

      }

      if (new_data_ready_MMC) {
            new_data_ready_MMC = false;

            // Drain the completed transaction from the SPI driver queue to free the hardware
            MMC_single_read();
            //Serial.printf("ICM Data: %.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n",ICM_Data_Holder.ax,ICM_Data_Holder.ay,ICM_Data_Holder.az,ICM_Data_Holder.gx,ICM_Data_Holder.gy,ICM_Data_Holder.gz);
            Serial.printf("MMC Data: %.4f\t%.4f\t%.4f\n",MMC_Data_Holder.mx,MMC_Data_Holder.my,MMC_Data_Holder.mz);
            
      }
}