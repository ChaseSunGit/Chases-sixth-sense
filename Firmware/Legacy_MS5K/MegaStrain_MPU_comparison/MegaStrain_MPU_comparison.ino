// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Wire.h>
#include "MegaStrain5000.h"
#include "MPU6050_MF3000.h"

#define CONTROL_LOOP_FREQUENCY 200 //This is the frequency of the low level control loop and is actuated by back IMU interrupt as the master timer
#define IMU_INTERRUPT_PIN 2 //This is the int pin connected to the IMU for timing the control loop via ISR

volatile bool backIMUready = false; // Flag set by ISR

// ISR for MPU6050 INT pin
void imuISR() {
  backIMUready = true;
}

void setup(void) {
  Serial.begin(921600);
  while (!Serial)
    delay(10); // Don't start until serial is ready

  Wire.begin();
  Wire.setClock(1000000);
  Wire1.begin();
  Wire1.setClock(1000000);
  Wire2.begin();
  Wire2.setClock(1000000);

  Serial.println("Initializing IMUs");
  MegaStrain_bulk_initialization(CONTROL_LOOP_FREQUENCY);
  mpu6050_bulk_initialization(CONTROL_LOOP_FREQUENCY);

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
    read_all_imus();

    //Kalman filter all IMUs
    //Kalman_batch_MS5K();

    //This block is used to check the time that has passed during read.
    Serial.printf("MS: %d\t", current_ms);
    int elapsed = micros()-start_time;
    Serial.printf("read took: %d\n",elapsed);

    //Printing for debugging
    for (int i = 0; i<(int)NUM_MS5K;i++){
      if(MS5K_Data_Holder[i].valid_flag == 1){
        //Serial.printf("IMU %d roll: %.4f\t pitch %.4f\n",i,MS5K_Data_Holder[i].roll,MS5K_Data_Holder[i].pitch);
        Serial.printf("IMU data ax: %.4f\t ay: %.4f\t az: %.4f\t gx: %.4f\t gy: %.4f\t gz: %.4f\n", MS5K_Data_Holder[i].ax, MS5K_Data_Holder[i].ay, MS5K_Data_Holder[i].az, MS5K_Data_Holder[i].gx, MS5K_Data_Holder[i].gy, MS5K_Data_Holder[i].gz);
      }
    }
    for (int i = 0; i<(int)NUM_IMUS;i++){
      if(IMU_Data_Holder[i].valid_flag == 1){
        //Serial.printf("IMU %d roll: %.4f\t pitch %.4f\n",i,IMU_Data_Holder[i].roll,IMU_Data_Holder[i].pitch);
        Serial.printf("IMU data ax: %.4f\t ay: %.4f\t az: %.4f\t gx: %.4f\t gy: %.4f\t gz: %.4f\n", IMU_Data_Holder[i].ax, IMU_Data_Holder[i].ay, IMU_Data_Holder[i].az, IMU_Data_Holder[i].gx, IMU_Data_Holder[i].gy, IMU_Data_Holder[i].gz);
      }
    }
    int kalman_elapsed = micros()-start_time-elapsed;
    Serial.printf("read took: %d\n",kalman_elapsed);

    
  }
}