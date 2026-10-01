#include "Sensor_Fusion.h"


Full_IMU_Data_t Full_IMU_Data
/**
 * \brief Performs the full sensor fusion with available IMU measurements
 * 
 * \param mode select the data to be computed using a bit array
                  MSB
                  7 - not used
                  6 - not used
                  5 - not used
                  4 - not used
                  3 - Quaternions
                  2 - Eulers
                  1 - Magnetometer
                  0 - Accelerometer + Gyroscope
                  LSB
 * \return true if sensor fusion was performed correctly
 */
bool Update_Sensor_Fusion(uint8_t mode) {
    // NOTE: Insert your preferred Madgwick or Mahony algorithm math here.
    // This updates Fusion_Data.q[0...3].

    // Convert Quaternions to Euler Angles (Standard Aerospace Sequence)
    float w = Fusion_Data.q[0];
    float x = Fusion_Data.q[1];
    float y = Fusion_Data.q[2];
    float z = Fusion_Data.q[3];

    Fusion_Data.roll  = atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * 57.29578f;
    Fusion_Data.pitch = asin(2.0f * (w * y - z * x)) * 57.29578f;
    Fusion_Data.yaw   = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * 57.29578f;
}