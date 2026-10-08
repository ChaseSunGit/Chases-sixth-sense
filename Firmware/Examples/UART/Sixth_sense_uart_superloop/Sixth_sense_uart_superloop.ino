#include <Arduino.h>
#include "SixthSense_IMU.h"

SixthSense_IMU imu;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // 1. Get configurations
    ICM_Config_t icm_cfg;
    MMC_Config_t mmc_cfg;
    Fusion_Config_t fusion_cfg;
    imu.returnDefaultConfig(icm_cfg, mmc_cfg, fusion_cfg);

    // 2. Initialize Hardware
    if (!imu.sensor_init(icm_cfg, mmc_cfg, fusion_cfg)) {
        Serial.println("IMU Init Failed!");
        while (1) delay(100);
    }

    // 3. Calibrate (works flawlessly because flags are propagated and task isn't blocking bus)
    Serial.println("Calibrating IMU...");
    imu.calibrateAccel(200);
    imu.calibrateGyro(200);

    // 4. Fire up the RTOS task natively from the object
    if (!imu.startRTOS_Task(1, 5)) {
        Serial.println("Failed to start IMU Task!");
    } else {
        Serial.println("IMU RTOS Task Running.");
    }
}

void loop() {
    // You can safely read data here from the wrapper whenever you want
    Fusion_Data_t fd = imu.getFusionData();
    Serial.printf("accel x: %.4f\ty: %.4f\tz: %.4f\n", fd.accel[0], fd.accel[1], fd.accel[2]);
    Serial.printf("gyro  x: %.4f\ty: %.4f\tz: %.4f\n", fd.gyro[0], fd.gyro[1], fd.gyro[2]);
    Serial.printf("mag   x: %.4f\ty: %.4f\tz: %.4f\n", fd.mag[0], fd.mag[1], fd.mag[2]);
    Serial.printf("Roll: %.4f\tPitch:\t%.4f\tYaw: %.4f\n", fd.roll, fd.pitch, fd.yaw);
    
    vTaskDelay(pdMS_TO_TICKS(1)); // Print at up to 1000hz
}