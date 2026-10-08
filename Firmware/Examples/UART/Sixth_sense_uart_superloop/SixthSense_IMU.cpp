/**
 * \file SixthSense_IMU.cpp
 * \brief Implementation of the unified SixthSense_IMU wrapper.
 */

#include "SixthSense_IMU.h"

// Define the static pointer to allow static ISR functions to access class member data
SixthSense_IMU* SixthSense_IMU::instance = nullptr;

SixthSense_IMU::SixthSense_IMU() 
            : icm(), 
              mmc(), 
              data_holder{} {
      
      // Initialize Fusion structure
      FusionAhrsInitialise(&ahrs);
      FusionBiasInitialise(&bias);
      instance = this;//initialize object for ISR since it cannot access the hidden "this" object in function argument
}

//ISR for data ready interrupt from chip
void IRAM_ATTR SixthSense_IMU::ISR_dataReady() {
      if (!instance) return;

      //Force subclass data ready variables to be toggled so that calibration functions can work without overall RTOS task
      instance->icm.force_data_ready();
      instance->mmc.force_data_ready();

      instance->new_data_ready = true;

      // RTOS implementation. If imu_task_handle is not set, the code will run like superloop
      if (instance->imu_task_handle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(instance->imu_task_handle, &xHigherPriorityTaskWoken);//Marks the IMU_task_handle as ready
            
            if (xHigherPriorityTaskWoken) {
                  portYIELD_FROM_ISR();//If its higher priority (it will be when sensor is idle), immediately yield to the reader task
            }
      }
}

/**
 * @brief Validates the configuration bounds for sensor fusion.
 * 
 * @param config Pointer to the Fusion_Config_t instance to validate.
 * @return true if all parameters are within valid ranges, false otherwise.
 */
bool SixthSense_IMU::check_config_validity(const Fusion_Config_t &config) {

      bool is_valid = true;

      // 1. axis_setting: Valid options are 1, 2, or 3
      if (config.axis_setting < 1 || config.axis_setting > 3) {
            printf("[Fusion] axis_setting: %u (Expected: 1, 2, or 3)\n", 
                  config.axis_setting);
            is_valid = false;
      }

      // 2. fusion_gain: Normalized weighting factor between 0.0f and 1.0f
      if (isnan(config.fusion_gain) || config.fusion_gain < 0.0f || config.fusion_gain > 1.0f) {
            printf("[Fusion] fusion_gain: %f (Expected: [0.0, 1.0])\n", 
                  config.fusion_gain);
            is_valid = false;
      }

      // 3. accel_rejection: Angular deviation threshold in degrees (0.0 to 180.0)
      if (isnan(config.accel_rejection) || config.accel_rejection <= 0.0f || config.accel_rejection > 180.0f) {
            printf("[Fusion] accel_rejection: %f deg (Expected: (0.0, 180.0])\n", 
                  config.accel_rejection);
            is_valid = false;
      }

      // 4. mag_rejection: Angular deviation threshold in degrees (0.0 to 180.0)
      if (isnan(config.mag_rejection) || config.mag_rejection <= 0.0f || config.mag_rejection > 180.0f) {
            printf("[Fusion] mag_rejection: %f deg (Expected: (0.0, 180.0])\n", 
                  config.mag_rejection);
            is_valid = false;
      }

      // 5. recovery_period: Time duration after rejection (must be strictly non-negative, reasonable max e.g. 60s)
      if (isnan(config.recovery_period) || config.recovery_period < 0.0f || config.recovery_period > 60.0f) {
            printf("[Fusion] recovery_period: %f s (Expected: [0.0, 60.0])\n", 
                  config.recovery_period);
            is_valid = false;
      }

      // 6. gyro_stationary_threshold: Gyroscope motion threshold in deg/s (must be non-negative)
      if (isnan(config.gyro_stationary_threshold) || config.gyro_stationary_threshold < 0.0f || config.gyro_stationary_threshold > 50.0f) {
            printf("[Fusion] gyro_stationary_threshold: %f dps (Expected: [0.0, 50.0])\n", 
                  config.gyro_stationary_threshold);
            is_valid = false;
      }

      // 7. gyro_stationary_period: Time duration to detect stationary state (must be positive)
      if (isnan(config.gyro_stationary_period) || config.gyro_stationary_period <= 0.0f || config.gyro_stationary_period > 600.0f) {
            printf("[Fusion] gyro_stationary_period: %f s (Expected: (0.0, 600.0])\n", 
                  config.gyro_stationary_period);
            is_valid = false;
      }

      return is_valid;
}

bool SixthSense_IMU::sensor_init(const ICM_Config_t &icm_cfg, const MMC_Config_t &mmc_cfg, const Fusion_Config_t &fusion_cfg) {

      //Check config validity

      if (!check_config_validity(fusion_cfg)){
            return false;
      }

      //Initialize sensors
      bool icm_ready = icm.init_chip(icm_cfg);
      bool mmc_ready = mmc.init_chip(mmc_cfg);

      //Configure sensor fusion
      fusion_enabled = fusion_cfg.fusion_enable & icm_ready; //Do not enable sensor fusion if ICM data is not present as fusion cannot be performed with mag only

      desired_axis_definition = fusion_cfg.axis_setting;

      if (fusion_enabled){

            float sample_rate = icm.getODR();

            FusionBiasSettings bias_settings = {
                  .sampleRate = sample_rate,
                  .stationaryThreshold = fusion_cfg.gyro_stationary_threshold,
                  .stationaryPeriod = fusion_cfg.gyro_stationary_period
            };

            float gyro_dps_range = (float)ICM42607::gyro_range_table[icm_cfg.gyro_range - 1];

                  // Configure x-io AHRS settings based on initialization
            FusionAhrsSettings settings = {
                  .sampleRate = sample_rate,
                  .convention = FusionConventionNwu,
                  .gain = fusion_cfg.fusion_gain,
                  .gyroscopeRange = gyro_dps_range,
                  .accelerationRejection = fusion_cfg.accel_rejection,
                  .magneticRejection = fusion_cfg.mag_rejection,
                  .rejectionTimeout = fusion_cfg.recovery_period
            };

            FusionAhrsSetSettings(&ahrs, &settings);
      
      }

      //Latch the data ready interrupt on either ICM (default) or MMC (only when ICM is off but MMC is on)
      if (!icm_ready && mmc_ready){
            attachInterrupt(digitalPinToInterrupt(PIN_INT_MMC), ISR_dataReady, RISING);
      }
      else{
            attachInterrupt(digitalPinToInterrupt(PIN_INT_ICM), ISR_dataReady, RISING);
      }
      return fusion_enabled;
}

bool SixthSense_IMU::processSensorData() {
      new_data_ready = false;
      //New data ready can be used with superloop if the rtos object is not initalized
      //Just continuously poll new_data_ready in loop and if true run processSensorData

      // Attempt single_reads. These unpack the DMA buffers if data is flagged ready
      bool icm_new = icm.single_read();
      bool mmc_new = mmc.single_read();

      if (!icm_new && !mmc_new){
            return false;
      }//No unique data coming in

      data_holder.msTimeStamp = millis();

      ICM_Data_t icm_data = getICMData();
      MMC_Data_t mmc_data = getMMCData();
      
      //First flip axis to desired coordinate system
      if (desired_axis_definition == 1){//default, no flip
            data_holder.accel[0] = icm_data.accel_cal[0];
            data_holder.accel[1] = icm_data.accel_cal[1];
            data_holder.accel[2] = icm_data.accel_cal[2];

            data_holder.gyro[0] = icm_data.gyro_cal[0];
            data_holder.gyro[1] = icm_data.gyro_cal[1];
            data_holder.gyro[2] = icm_data.gyro_cal[2];

            data_holder.mag[0] = mmc_data.mag_cal[0];
            data_holder.mag[1] = mmc_data.mag_cal[1];
            data_holder.mag[2] = mmc_data.mag_cal[2];
      }
      else if (desired_axis_definition == 2){// Microstrain default, x unchanged, y flips negative, z flips negative
            data_holder.accel[0] = icm_data.accel_cal[0];
            data_holder.accel[1] = -1.0f * icm_data.accel_cal[1];
            data_holder.accel[2] = -1.0f * icm_data.accel_cal[2];

            data_holder.gyro[0] = icm_data.gyro_cal[0];
            data_holder.gyro[1] = -1.0f * icm_data.gyro_cal[1];
            data_holder.gyro[2] = -1.0f * icm_data.gyro_cal[2];

            data_holder.mag[0] = mmc_data.mag_cal[0];
            data_holder.mag[1] = -1.0f * mmc_data.mag_cal[1];
            data_holder.mag[2] = -1.0f * mmc_data.mag_cal[2];
      }
      else if (desired_axis_definition == 3) {// Microstrain upright, measured z maps to output neg x, measured x maps to z, y flips negative 
            data_holder.accel[2] = icm_data.accel_cal[0];
            data_holder.accel[1] = -1.0f * icm_data.accel_cal[1];
            data_holder.accel[0] = -1.0f * icm_data.accel_cal[2];

            data_holder.gyro[2] = icm_data.gyro_cal[0];
            data_holder.gyro[1] = -1.0f * icm_data.gyro_cal[1];
            data_holder.gyro[0] = -1.0f * icm_data.gyro_cal[2];

            data_holder.mag[2] = mmc_data.mag_cal[0];
            data_holder.mag[1] = -1.0f * mmc_data.mag_cal[1];
            data_holder.mag[0] = -1.0f * mmc_data.mag_cal[2];
      }
      else{
            Serial.println("[Axis setting] Fusion axis setting out of bounds. Limit 1-3.");
      }
      
      if (!fusion_enabled){
            return true; //No fusion required, only raw data desired
      }

      //Perform fusion

      FusionVector gyroscope = {
            data_holder.gyro[0], 
            data_holder.gyro[1], 
            data_holder.gyro[2]
      };

      FusionVector accelerometer = {
            data_holder.accel[0], 
            data_holder.accel[1], 
            data_holder.accel[2]
      };

      FusionVector magnetometer = {
            data_holder.mag[0], 
            data_holder.mag[1], 
            data_holder.mag[2]
      };

      gyroscope = FusionBiasUpdate(&bias, gyroscope); //Update bias to compensate for stationary gyro drift

      if (!mmc_new){
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);
      }
      else{
            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer);
      }

      //Now compute quatornians

      FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);

      // Note: In x-io Fusion, elements are accessed via quaternion.element.w, .x, .y, .z
      data_holder.q[0] = quaternion.element.w;
      data_holder.q[1] = quaternion.element.x;
      data_holder.q[2] = quaternion.element.y;
      data_holder.q[3] = quaternion.element.z;

      // 2. Convert the quaternion to Euler angles (in degrees)
      FusionEuler euler = FusionQuaternionToEuler(quaternion);

      data_holder.roll  = euler.angle.roll;   // Rotation around X axis (-180 to +180 deg)
      data_holder.pitch = euler.angle.pitch;  // Rotation around Y axis (-90 to +90 deg)
      data_holder.yaw   = euler.angle.yaw;

      return true;
}

// Pass-through wrapper functions
bool SixthSense_IMU::calibrateAccel(int num_samples) {
      return icm.accel_calib(num_samples);
}

bool SixthSense_IMU::calibrateGyro(int num_samples) {
      return icm.gyro_calib(num_samples);
}

bool SixthSense_IMU::calibrateMag(uint8_t num_seconds, uint8_t num_timeout) {
      return mmc.Calibrate_Full_Soft_Hard_Iron(num_seconds, num_timeout);
}

ICM_Data_t SixthSense_IMU::getICMData() const {
      return icm.getData();
}

MMC_Data_t SixthSense_IMU::getMMCData() const {
      return mmc.getData();
}

Fusion_Data_t SixthSense_IMU::getFusionData() const {
      return data_holder;
}

bool SixthSense_IMU::readICMconfig() {
      ICM_Config_t read_config;
      return icm.read_config(read_config);
}

bool SixthSense_IMU::readMMCconfig() {
      MMC_Config_t read_config;
      return mmc.read_config(read_config);
}

//Input 3 config structs to receive their default values, used to reset chip settings to factory default
void returnDefaultConfig(ICM_Config_t &icm_cfg, MMC_Config_t &mmc_cfg, Fusion_Config_t &fusion_cfg){

      Serial.println("[Config] ICM default config:");
      icm_cfg = {
            .outputRate = 5,    //1-8: 1 (12.5Hz), 2 (25Hz), 3 (50Hz), 4 (100Hz), 5 (200Hz, default), 6 (400Hz), 7(800Hz), 8(1600Hz)
            .accel_bw = 5,     //1-8: 1 (16Hz), 2 (25Hz), 3 (34Hz), 4 (53Hz), 5 (73Hz, default), 6 (121Hz), 7(180Hz), 8(Bypassed)
            .gyro_bw = 5,      //1-8: 1 (16Hz), 2 (25Hz), 3 (34Hz), 4 (53Hz), 5 (73Hz, default), 6 (121Hz), 7(180Hz), 8(Bypassed)
            .accel_range = 3,  //1-4: 1 (±2g), 2 (±4g), 3 (±8g, default), 4 (±16g)
            .gyro_range = 3,   //1-4: 1 (±250dps), 2 (±500dps), 3 (±1000dps, default), 4 (±2000dps)    
            .chip_enable = 1,  //Switch to toggle ICM: 0 - disable, 1 - enable
      };

      Serial.println("[Config] MMC default config:");
      mmc_cfg = {
            .outputRate = 6,        //1-7: 1 (1Hz), 2 (10Hz), 3 (20Hz), 4 (50Hz), 5 (100Hz), 6 (200Hz, default), 7(1000Hz)
            .bandwidth = 1,         //1-4: 1 (100Hz, default), 2 (200Hz), 3 (400Hz), 4 (800Hz)
            .setFrequency = 8,      //0-8 (measurements per set): 0: (disable autoset), 1 (1 sample), 2 (25), 3 (75), 4 (100), 5 (250), 6 (500), 7 (1000), 8 (2000, default)
            .chip_enable = 1        //0 - disable, 1 - enable
      };

      Serial.println("[Config] Fusion default config:");
      fusion_cfg = {
            .fusion_enable = 1,           //Enable sensor fusion operation
            .axis_setting = 2,            //Transforming xyz axis for desired stance. 1 (Standard), 2 (Microstrain, default), 3 (Microstrain upright)

            .fusion_gain = 0.2,           //0-1, lower biases towards gyro and higher towards accel. 0.2 default
            .accel_rejection = 20.0f,     //Threshold of deviation between instantaneous accel output and fusion output to reject high linear acceleration peaks, 20 deg default
            .mag_rejection = 10.0f,       //Threshold of deviation between instantaneous mag output and fusion output to reject burst magnetic field distortions, 10 deg default
            .recovery_period = 0.5f,      //How long after rejection do we trust accel / mag, default 0.5s

            .gyro_stationary_threshold = 3.0f,  //Threshold for how low the gyroscope reading should be before being treated as stationary, default 3dps
            .gyro_stationary_period = 3.0f      //How long to wait for gyro to stay within range before reseting offsets, default 3s
      };
}

// Rtos task initialization
bool SixthSense_IMU::RTOS_startReadingTask(BaseType_t core_id, UBaseType_t priority) {
      if (imu_task_handle != NULL) return true; // Already running

      BaseType_t result = xTaskCreatePinnedToCore(
            IMU_Task_Trampoline, 
            "IMU_Task", 
            8192, 
            this,          // Pass the class instance as the parameter
            priority, 
            &imu_task_handle, 
            core_id
      );
      return (result == pdPASS);
}

// T
void SixthSense_IMU::IMU_Task_Trampoline(void *pvParameters) {
      // Cast the void pointer back to our object instance
      SixthSense_IMU *imu = static_cast<SixthSense_IMU*>(pvParameters);

      while (1) {
            // Block indefinitely until notified by the ISR
            // pdTRUE clears the notification value to 0 on exit
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Fetch and process data
            imu->processSensorData();
      }
}