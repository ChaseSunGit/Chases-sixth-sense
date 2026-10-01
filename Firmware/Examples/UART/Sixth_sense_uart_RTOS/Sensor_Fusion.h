#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <math.h>
#include "ICM42607_Driver.cpp"
#include "ICM42607_Driver.cpp"

struct Full_IMU_Data_t {
      //Processed IMU sensor measurements
      float accel[3];//xyz
      float gyro[3];//xyz
      float mag[3];//xyz

      
      float q[4]; // Quaternion [w, x, y, z]
      float roll;
      float pitch;
      float yaw;
};

extern Full_IMU_Data_t Full_IMU_Data;

void Update_Sensor_Fusion(uint8_t mode);

#endif