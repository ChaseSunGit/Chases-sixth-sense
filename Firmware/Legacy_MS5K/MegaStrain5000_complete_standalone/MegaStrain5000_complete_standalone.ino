// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Wire.h>
#include "MegaStrain5000.h"

#define CONTROL_LOOP_FREQUENCY 200 //This is the frequency of the low level control loop and is actuated by back IMU interrupt as the master timer
#define IMU_INTERRUPT_PIN 2 //This is the int pin connected to the IMU for timing the control loop via ISR

//Settings
bool verboseSerial = false; //the full serial out message for debugging
bool plotSerial = true; //Only outputs a line of comma separated values


volatile bool backIMUready = false; // Flag set by ISR

// ISR for MPU6050 INT pin
void imuISR() {
  backIMUready = true;
}

void setup(void) {
  Serial.begin(921600);
  //while (!Serial)
    //delay(10); // Don't start until serial is ready

  Wire.begin();
  Wire.setClock(400000);
  Wire1.begin();
  Wire1.setClock(400000);
  Wire2.begin();
  Wire2.setClock(400000);

  Serial.println("Initializing IMUs");
  MegaStrain_bulk_initialization(CONTROL_LOOP_FREQUENCY);

  //Enable the interrupt on the back IMU
  pinMode(IMU_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuISR, RISING);//Attach the int pin of the IMU to the ISR
  
  
  Serial.println("Calibrating: do not move sensors for 2 seconds");
  ICM_bulk_calibration(400,backIMUready);//Last step: calibrate sensors


  //Calibrate accel. This line will block code from advancing forward so comment out for normal operation
  //mpu6050_factory_accel_calibration(3,400,backIMUready);
  
  Serial.println("Calibration completed. Pausing for 1 second");
  delay(1000);
}

void loop() {
  //if(false){
  if (backIMUready) {
    backIMUready = false; // Clear flag

    //Used for checking timing
    unsigned long current_ms = millis();
    unsigned long start_time = micros();

    //Now make a read
    read_all_MS5K();
    //read_all_MS5K(true); //Set calibration mode as constantly on, for magnetometer calibration

    //Kalman filter all IMUs
    Kalman_batch_MS5K(1);//mode selector: 0 means normal (microstrain) and 1 means upright. Some axis are flipped

    //This block is used to check the time that has passed during read.
    
    int elapsed = micros()-start_time;
    

    //Printing for debugging
    if (verboseSerial){
      Serial.printf("MS: %d\t", current_ms);
      Serial.printf("read took: %d\n",elapsed);
      for (int i = 0; i<(int)NUM_MS5K;i++){
        if(MS5K_Data_Holder[i].valid_flag == 1){
          Serial.printf("IMU %d roll: %.4f\t pitch: %.4f\t yaw: %.4f\n",i,MS5K_Data_Holder[i].roll,MS5K_Data_Holder[i].pitch,MS5K_Data_Holder[i].yaw);
          Serial.printf("ICM data ax: %.4f\t ay: %.4f\t az: %.4f\t gx: %.4f\t gy: %.4f\t gz: %.4f\n", MS5K_Data_Holder[i].ax, MS5K_Data_Holder[i].ay, MS5K_Data_Holder[i].az, MS5K_Data_Holder[i].gx, MS5K_Data_Holder[i].gy, MS5K_Data_Holder[i].gz);
          Serial.printf("MLX %d data mx: %.4f\t my: %.4f\t mz: %.4f\n", i, MS5K_Data_Holder[i].mx, MS5K_Data_Holder[i].my, MS5K_Data_Holder[i].mz);
          
          //The following is for calibration purposes. To perform magnetometer calibration, uncomment the following code
          // and set calibration mode to true when calling read_all_MS5L(). Rotate the IMUs through their entire range of motion slowly
          // until the max and min values stop changing. Record the hardiron and softiron calibration values and populate the calibration arrays
          // in the MegaStrain5000 header file.

          /*
          
          Serial.printf("MLX ranges mx: %.4f\t %.4f\t my: %.4f\t %.4f\t mz: %.4f\t %.4f\n", MS5K_Data_Holder[i].mx_max,MS5K_Data_Holder[i].mx_min, MS5K_Data_Holder[i].my_max, MS5K_Data_Holder[i].my_min, MS5K_Data_Holder[i].mz_max, MS5K_Data_Holder[i].mz_min);
        
          //This code is to calculate the softiron and hardiron calibrations for the mag
          //hard iron is simply treated as offset while softiron is treated as scaling for normalization
          float mx_hardiron = (MS5K_Data_Holder[i].mx_max + MS5K_Data_Holder[i].mx_min)/2;
          float my_hardiron = (MS5K_Data_Holder[i].my_max + MS5K_Data_Holder[i].my_min)/2;
          float mz_hardiron = (MS5K_Data_Holder[i].mz_max + MS5K_Data_Holder[i].mz_min)/2;
          float mx_softiron = 2/(MS5K_Data_Holder[i].mx_max - MS5K_Data_Holder[i].mx_min); //normalize the whole range to -1 to 1
          float my_softiron = 2/(MS5K_Data_Holder[i].my_max - MS5K_Data_Holder[i].my_min);
          float mz_softiron = 2/(MS5K_Data_Holder[i].mz_max - MS5K_Data_Holder[i].mz_min);

          Serial.printf("MLX %d calib mx_hardiron: %.4f\t my_hardiron: %.4f\t mz_hardiron: %.4f\t mx_softiron: %.4f\t my_softiron: %.4f\t mz_softiron: %.4f\n", i,mx_hardiron,my_hardiron,mz_hardiron,mx_softiron,my_softiron,mz_softiron);
        

          Serial.printf("MLX normalized mx: %.4f\t my: %.4f\t mz: %.4f\n", (MS5K_Data_Holder[i].mx-mx_hardiron)*mx_softiron, (MS5K_Data_Holder[i].my-my_hardiron)*my_softiron, (MS5K_Data_Holder[i].mz-mz_hardiron)*mz_softiron);
          */

        }
      }
      int kalman_elapsed = micros()-start_time-elapsed;
      Serial.printf("read took: %d\n",kalman_elapsed);
    }
    if (plotSerial){
      for (int i = 0; i<(int)NUM_MS5K;i++){
        if(MS5K_Data_Holder[i].valid_flag == 1){
          //y -> x, x -> -y
          Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,",MS5K_Data_Holder[i].ay, (-1.0)*MS5K_Data_Holder[i].ax, MS5K_Data_Holder[i].az, MS5K_Data_Holder[i].gy*0.0174532925, (-1.0)*MS5K_Data_Holder[i].gx*0.0174532925, MS5K_Data_Holder[i].gz*0.0174532925);
          Serial.printf("%.4f,%.4f,%.4f,",MS5K_Data_Holder[i].my, (-1.0)*MS5K_Data_Holder[i].mx, MS5K_Data_Holder[i].mz);
          Serial.printf("%.4f,%.4f,%.4f,",MS5K_Data_Holder[i].roll*0.0174532925, MS5K_Data_Holder[i].pitch*0.0174532925, MS5K_Data_Holder[i].yaw*0.0174532925);
        }
      }
      Serial.println();
    }
  }
}