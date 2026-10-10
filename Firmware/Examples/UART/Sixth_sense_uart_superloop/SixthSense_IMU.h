/**
 * \file SixthSense_IMU.h
 * \brief Unified wrapper for ICM42607 and MMC5983MA with x-io Fusion integration.
 */

#ifndef SIXTHSENSE_IMU_H
#define SIXTHSENSE_IMU_H

#include <Fusion.h> // x-io Technologies Fusion library https://github.com/xioTechnologies/Fusion
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "ICM42607_Driver.h"
#include "MMC5983MA_Driver.h"

struct Fusion_Data_t {
      float accel[3];         //Accelerometer xyz
      float gyro[3];          //Gyroscope xyz
      float mag[3];           //Magnetometer xyz

      uint32_t msTimeStamp;   //32 bit milisecond timestamp
      float q[4];             //Quatornians

      //Eulers
      float roll;
      float pitch;
      float yaw;
};

struct Fusion_Config_t {
      bool fusion_enable; //Enable sensor fusion operation
      uint8_t axis_setting; //Transforming xyz axis for desired stance. 1 (Standard), 2 (Microstrain, default), 3 (Microstrain upright)

      float fusion_gain; //0-1, lower biases towards gyro and higher towards accel. 0.2 default
      float accel_rejection; //Threshold of deviation between instantaneous accel output and fusion output to reject high linear acceleration peaks, 20 deg default
      float mag_rejection; //Threshold of deviation between instantaneous mag output and fusion output to reject burst magnetic field distortions, 10 deg default
      float recovery_period; //How long after rejection do we trust accel / mag, default 0.5s

      float gyro_stationary_threshold; //Threshold for how low the gyroscope reading should be before being treated as stationary, default 3dps
      float gyro_stationary_period; //How long to wait for gyro to stay within range before reseting offsets, default 3s
};

class SixthSense_IMU {
public:
      // Constructor handles RTOS routing for both underlying sensors
      SixthSense_IMU();

      // Unified Initialization
      bool sensor_init(const ICM_Config_t &icm_cfg, const MMC_Config_t &mmc_cfg, const Fusion_Config_t &fusion_cfg);

      bool check_config_validity(const Fusion_Config_t &config);

      // Unified Processing Loop (Call in superloop or RTOS task)
      bool processSensorData();

      // Pass-throughs for explicit Subclass Functionality
      bool calibrateAccel(int num_samples = 2000);
      bool calibrateGyro(int num_samples = 2000);
      bool calibrateMag(uint8_t num_seconds = 6, uint8_t num_timeout = 36);

      // Read settings
      bool readICMconfig();
      bool readMMCconfig();

      // Getters for Raw and Calibrated Device Data
      ICM_Data_t getICMData() const;
      MMC_Data_t getMMCData() const;
      Fusion_Data_t getFusionData() const;

      //Calibration set and get passthroughs
      ICM_Cal_t getICMCal() const { return icm.getCal(); }
      MMC_Cal_t getMMCCal() const { return mmc.getCal(); }
      void setICMCal(ICM_Cal_t &cal) { icm.setCal(cal); }
      void setMMCCal(MMC_Cal_t &cal) { mmc.setCal(cal); }

      //Returns the default values of each of the configuration files
      static void returnDefaultConfig(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg);

      //Setup ISR for data ready
      static SixthSense_IMU* instance;
      static void IRAM_ATTR ISR_dataReady();

      //RTOS start the sensor handling task
      bool RTOS_startReadingTask(BaseType_t core_id = 1, UBaseType_t priority = 5);
      void RTOS_suspendReading() { if(imu_task_handle) vTaskSuspend(imu_task_handle); }
      void RTOS_resumeReading() { if(imu_task_handle) vTaskResume(imu_task_handle); }

private:
      ICM42607 icm;
      MMC5983MA mmc;

      Fusion_Data_t data_holder;

      //Boolean to mark if fusion is enabled
      bool fusion_enabled;

      //Axis definition
      uint8_t desired_axis_definition; //stance of output coordinate

      // x-io Fusion variables
      FusionAhrs ahrs;
      FusionBias bias; //gyro bias tracking
      uint32_t last_time_millis;

      //Data ready boolean
      volatile bool new_data_ready;

      //RTOS task handle for sensor reading
      TaskHandle_t imu_task_handle = NULL;
      // Static trampoline to route FreeRTOS to the class instance
      static void IMU_Task_Trampoline(void *pvParameters);

      static constexpr float GRAVITY_ICM = 9.81f; // Convert Gs to m/s^2
      static constexpr float DEG2RAD_ICM = 0.01745329f; // Convert dps to rad/s
};

#endif /* SIXTHSENSE_IMU_H */