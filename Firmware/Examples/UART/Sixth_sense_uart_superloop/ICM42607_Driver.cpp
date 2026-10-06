/**
 * \file      ICM42607_Driver.cpp
 * \brief     Library for interfacing with ICM42607.
 *
 * \authors   Chase Sun
 */

// INCLUDES
#include "ICM42607_Driver.h"

// Define the static pointer to allow static ISR functions to access class member data
ICM42607* ICM42607::instance = nullptr;

// Constructor initializing variables and binding the static instance pointer
ICM42607::ICM42607(bool use_rtos, TaskHandle_t sensor_task) 
            : spi_dma{},
              data_holder{},
              dma_in_progress(false), 
              new_data_ready(false), 
              IMU_first_read(false), 
              using_RTOS(use_rtos), 
              SensorTaskHandle(sensor_task) {

      instance = this;
}

// FUNCTIONS

//First two functions are a pair of quick ISR functions for data ready & DMA complete

//ISR for data ready interrupt from chip
void IRAM_ATTR ICM42607::ISR_dataReady() {
      if (instance && !instance->dma_in_progress) {//If the object exists and if dma is not currently in progress
            // Queue the pre-armed DMA transaction with zero tick delay
            if (spi_device_queue_trans(instance->spi_dma.handle, &instance->spi_dma.trans, 0) == ESP_OK) {
                  instance->dma_in_progress = true;
            }
      }

      //Triggers MMC read
      if (!dma_in_progress_MMC) {
            if (spi_device_queue_trans(SPI_DMA_MMC.handle, &SPI_DMA_MMC.trans, 0) == ESP_OK) {
                  dma_in_progress_MMC = true;
            }
      }
}


//Call back function when DMA completes transfer and for triggering data processing & communication
//RTOS version where an external sensor task handle is notified
void IRAM_ATTR ICM42607::ISR_DMAcomplete_callback(spi_transaction_t *trans) {
      if (!instance) return;
      instance->new_data_ready = true; // Raise flag for buffer full
      if (instance->using_RTOS){
            // Wake up the RTOS task
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (instance->SensorTaskHandle != NULL) {
                  vTaskNotifyGiveFromISR(instance->SensorTaskHandle, &xHigherPriorityTaskWoken);//This function checks if sensor task has higher priority than current task during isr firing
                  //If that is the case (should be in almost all cases as the sensor task is very high priority), the function will set the boolean to pdTrue
                  //Then, the ISR will exit and the sensor reading task is immediately executed before the current task is finished executing.
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
      }
}

/**
 * \brief Tool to write SPI command to a register
 *
 * \param reg //The SPI register to write to
 * \param data //The data to write to
 * \return
 */
void ICM42607::write_reg(uint8_t reg, uint8_t data) {
      spi_transaction_t t = {};
      t.flags = SPI_TRANS_USE_TXDATA;
      t.length = 16; //Two bytes
      t.tx_data[0] = reg;
      t.tx_data[1] = data;
      spi_device_polling_transmit(spi_dma.handle, &t);
}

/**
 * \brief Tool to read SPI register. This only reads 1 register for checking config. Do not use this to read actual Accel/Gyro/Mag data as its too slow
 *
 * \param reg //The SPI register to write to
 * \return
 */
uint8_t ICM42607::read_reg(uint8_t reg) {
      spi_transaction_t t = {};
      t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      t.length = 16;
      t.tx_data[0] = reg | SPI_READ_FLAG;
      t.tx_data[1] = 0x00; //send 0s so the read is valid
      spi_device_polling_transmit(spi_dma.handle, &t);
      return t.rx_data[1];
}

/**
 * \brief Function to initialize a single ICM42607
 *
 * \param ICM_Config configuration struct, consult ICM42607_Driver.h for details on config fields
 * \return boolean value true meaning successfully initialized and false failed
 */
bool ICM42607::init_chip(const ICM_Config_t &ICM_Config) {

      data_holder = {};//Empty out any holder value during initialization

      //First setup the SPI bus

      // Initialize the SPI bus
      if (!SPI_Bus_Init(PIN_MOSI_ICM, PIN_MISO_ICM, PIN_SCLK_ICM)) {
            Serial.println("[ICM-ERROR] SPI host initialization failed!");
            return 0;
      }

      // Add the specific SPI device
      if (!SPI_Add_Device(PIN_CS_ICM, ISR_DMAcomplete_callback, spi_dma.handle)) {
            Serial.println("[ICM-ERROR] Could not add ICM Device!");
            return 0;
      }

      // Initialize the SPI bus
      if (!SPI_Arm_DMA_Channel(ICM_BURST_LEN, (DATA_START_ICM  | SPI_READ_FLAG), spi_dma)) {
            Serial.println("[ICM-ERROR] DMA setup for ICM failed!");
            return 0;
      }

      //For configuration, we do 7 steps
      //1. set filter bandwidth
      //2. Set divider for reporting frequency - 200 hz default, controls the low level control loop speed if IMU mode is enabled
      //3. Set accel range - 8g default
      //4. Set gyro range - 1000dps default
      //5. Set interrupts. For non-back imus, enable int 2. for back imu enable 1 and 2
      //6. Set PWR_MGMT0 register to configure clock, gyro and accel in low noise mode mode
      //7. Check if the IMU id is what we expect - this checks if there is a valid connection to the IMU after all configuration

      //Before configuration, check validity of the struct passed in for out of bounds values
      if (!check_config_validity(ICM_Config)){
            return 0;
      }

      //First, We will use bitwise operations to construct the configuration for sample rate, bandwidth, and range of both accel and gyro
      uint8_t accel_ODR = 13 - ICM_Config.outputRate; //The accel odr ranges from 5 (1600 Hz) to 12 (12.5Hz) on the ACCEL_CONFIG0 register.
      uint8_t gyro_ODR  = 13 - ICM_Config.outputRate; //same as accel

      ODR = odr_table[ICM_Config.outputRate - 1];//index from 0

      uint8_t accel_range = (4 - ICM_Config.accel_range); //Accel full scale ranges from 0 (±16g) to 3 (±2g) left shifted 5 bits on ACCEL_CONFIG0
      data_holder.accel_conversion = ACCEL_LSB_PER_G[accel_range];
      uint8_t gyro_range = (4 - ICM_Config.gyro_range); //Gyro full scale ranges from 0 (±2000dps) to 3 (±250dps) left shifted 5 bits on GYRO_CONFIG0
      data_holder.gyro_conversion = GYRO_LSB_PER_DPS[gyro_range];

      uint8_t accel_bw = 8 - ICM_Config.accel_bw; //Bandwidth goes from 1 (180Hz) to 7 (16Hz) with 0 being no filter on Accel_CONFIG1
      uint8_t gyro_bw = 8 - ICM_Config.gyro_bw; //same as accel

      //Construct config0 for data rate and full scale range
      write_reg(ACCEL_CONFIG0_ICM,  (accel_ODR | (accel_range<<5)));
      write_reg(GYRO_CONFIG0_ICM, (gyro_ODR | (gyro_range<<5)));
      //Construct config1 for bandwidth
      write_reg(ACCEL_CONFIG1_ICM,  accel_bw);
      write_reg(GYRO_CONFIG1_ICM,  gyro_bw);

      //Set temperature bandwidith
      write_reg(TEMP_CONFIG0_ICM,  0x04);//16 hz for filtering temperature

      //Set up the interrupts
      //Set in pulse mode, push pull, active high, 0b00 011011 0x1B
      write_reg(INT_CONFIG_ICM, 0x1B);
      
      //Set int 2, 0b00001000, data ready
      //write_reg(INT_SOURCE3_ICM, 0x08);
      
      //Set int 1, 0b00001000, data ready
      write_reg(INT_SOURCE0_ICM, 0x08);

      //Set power mode
      // 0b0000 1111, low noise mode for both accel and gyro
      write_reg(PWR_MGMT0_ICM, 0x0F);
      
      //Check if we are communicating with the right chip
      uint8_t check_addr = read_reg(WHO_AM_I_ICM);
      if (check_addr != 0x60){
            Serial.printf("[ICM-ERROR] Wrong address on ICM! Address found to be %x, address we are looking for is 0x60\n", check_addr);
            return false; //Checking who am I failed, IMU not initialized
      }
      else{
            Serial.printf("[ICM] Correct address on ICM, found to be %x.\n",check_addr);
      }

      //Finally, echo the configuration back
      ICM_Config_t out_config;//Holder of read back config values
      read_config(out_config);

      Serial.println("[ICM] ICM ready to be deployed\n");
      return true;
}

/**
 * \brief Validates configuration struct fields and reports any invalid values.
 * \param config Configuration structure to check.
 * \return true if all fields are valid, false otherwise.
 */
bool ICM42607::check_config_validity(const ICM_Config_t &config) {
      bool is_valid = true;

      // outputRate: 1 (12.5Hz) to 8 (1600Hz)
      if (config.outputRate < 1 || config.outputRate > 8) {
            Serial.printf("[ICM-CONFIG-ERROR] Invalid outputRate: %u (Expected 1 - 8)\n", config.outputRate);
            is_valid = false;
      }

      // accel_range: 1 (±2g) to 4 (±16g)
      if (config.accel_range < 1 || config.accel_range > 4) {
            Serial.printf("[ICM-CONFIG-ERROR] Invalid accel_range: %u (Expected 1 - 4)\n", config.accel_range);
            is_valid = false;
      }

      // gyro_range: 1 (±250dps) to 4 (±2000dps)
      if (config.gyro_range < 1 || config.gyro_range > 4) {
            Serial.printf("[ICM-CONFIG-ERROR] Invalid gyro_range: %u (Expected 1 - 4)\n", config.gyro_range);
            is_valid = false;
      }

      // accel_bw: 1 (16Hz) to 8 (Bypassed)
      if (config.accel_bw < 1 || config.accel_bw > 8) {
            Serial.printf("[ICM-CONFIG-ERROR] Invalid accel_bw: %u (Expected 1 - 8)\n", config.accel_bw);
            is_valid = false;
      }

      // gyro_bw: 1 (16Hz) to 8 (Bypassed)
      if (config.gyro_bw < 1 || config.gyro_bw > 8) {
            Serial.printf("[ICM-CONFIG-ERROR] Invalid gyro_bw: %u (Expected 1 - 8)\n", config.gyro_bw);
            is_valid = false;
      }

    return is_valid;
}

/**
 * \brief Reads the set data rate, bandwidth, and range of both accel and gyro from device registers
 * \param out_config reference to struct where decoded values will be populated
 * \return boolean indicating success
 */
bool ICM42607::read_config(ICM_Config_t &out_config) {
      // Read current config registers
      uint8_t accel_conf0 = read_reg(ACCEL_CONFIG0_ICM);
      uint8_t gyro_conf0  = read_reg(GYRO_CONFIG0_ICM);
      uint8_t accel_conf1 = read_reg(ACCEL_CONFIG1_ICM);
      uint8_t gyro_conf1  = read_reg(GYRO_CONFIG1_ICM);

      // Extract ODR (bits 3:0): ODR = 13 - code
      out_config.outputRate = 13 - (accel_conf0 & 0x0F);

      // Extract full scale range (bits 6:5): range = 4 - code
      out_config.accel_range = 4 - ((accel_conf0 >> 5) & 0x03);
      out_config.gyro_range  = 4 - ((gyro_conf0 >> 5) & 0x03);

      // Extract bandwidth (bits 2:0): bw = 8 - code
      out_config.accel_bw = 8 - (accel_conf1 & 0x07);
      out_config.gyro_bw  = 8 - (gyro_conf1 & 0x07);

      return true;
}

static void ICM42607::parse_config(const ICM_Config_t &config) {

      Serial.println("--- ICM-42607 Current Configuration ---");

      // Output Data Rate
      if (config.outputRate >= 1 && config.outputRate <= 8) {
            Serial.printf("Output Data Rate (ODR) : %.1f Hz\n", odr_table[config.outputRate - 1]);
      } else {
            Serial.printf("Output Data Rate (ODR) : Invalid (%u)\n", config.outputRate);
      }

      // Accelerometer Range & Bandwidth
      if (config.accel_range >= 1 && config.accel_range <= 4) {
            Serial.printf("Accel Full-Scale Range : +/-%d g\n", accel_range_table[config.accel_range - 1]);
      } else {
            Serial.printf("Accel Full-Scale Range : Invalid (%u)\n", config.accel_range);
      }

      if (config.accel_bw >= 1 && config.accel_bw <= 8) {
            int bw = bw_table[config.accel_bw - 1];
            if (bw == 0) Serial.println("Accel Filter Bandwidth : Bypassed");
            else         Serial.printf("Accel Filter Bandwidth : %d Hz\n", bw);
      } else {
            Serial.printf("Accel Filter Bandwidth : Invalid (%u)\n", config.accel_bw);
      }

      // Gyroscope Range & Bandwidth
      if (config.gyro_range >= 1 && config.gyro_range <= 4) {
            Serial.printf("Gyro Full-Scale Range  : +/-%d dps\n", gyro_range_table[config.gyro_range - 1]);
      } else {
            Serial.printf("Gyro Full-Scale Range  : Invalid (%u)\n", config.gyro_range);
      }

      if (config.gyro_bw >= 1 && config.gyro_bw <= 8) {
            int bw = bw_table[config.gyro_bw - 1];
            if (bw == 0) Serial.println("Gyro Filter Bandwidth  : Bypassed");
            else         Serial.printf("Gyro Filter Bandwidth  : %d Hz\n", bw);
      } else {
            Serial.printf("Gyro Filter Bandwidth  : Invalid (%u)\n", config.gyro_bw);
      }
      Serial.println("---------------------------------------");
}

/**
 * \brief Function to process DMA output from interrupt. At this point, the data is loaded into the dma_rx_ICM buffer and is ready to be read
 * \return boolean whether read was successful
 */
bool ICM42607::single_read(){

      spi_transaction_t *r_trans;

      if (spi_device_get_trans_result(spi_dma.handle, &r_trans, 0) != ESP_OK) {
            return false;
      }

      dma_in_progress = false;
      
      // Make the read

      // Unpack raw 16-bit values
      int16_t temp_raw = (int16_t)((spi_dma.rx_buffer[1]  << 8) | spi_dma.rx_buffer[2]);

      data_holder.temp  = ((float)temp_raw / 128.0f) + 25.0f;

      int16_t accel_raw[3];
      int16_t gyro_raw[3];
      
      accel_raw[0] = (int16_t)((spi_dma.rx_buffer[3]  << 8) | spi_dma.rx_buffer[4]);
      accel_raw[1] = (int16_t)((spi_dma.rx_buffer[5]  << 8) | spi_dma.rx_buffer[6]);
      accel_raw[2] = (int16_t)((spi_dma.rx_buffer[7]  << 8) | spi_dma.rx_buffer[8]);
      
      gyro_raw[0] = (int16_t)((spi_dma.rx_buffer[9]  << 8) | spi_dma.rx_buffer[10]);
      gyro_raw[1] = (int16_t)((spi_dma.rx_buffer[11] << 8) | spi_dma.rx_buffer[12]);
      gyro_raw[2] = (int16_t)((spi_dma.rx_buffer[13] << 8) | spi_dma.rx_buffer[14]);

      //Convert accel and gyro values with scaling

      for (int i = 0; i < 3; i++){
            data_holder.accel[i] = accel_raw[i] / data_holder.accel_conversion * GRAVITY_ICM;
            data_holder.gyro[i] = gyro_raw[i] / data_holder.gyro_conversion * DEG2RAD_ICM;
      }
      
      //Apply calibration
      apply_calibration();
      //Optional, apply temp correction

      return true;
}

/**
 * \brief Function to apply calibration offsets to accel and gyro values
 * \return nothing
 */
void ICM42607::apply_calibration(){
      for (int i = 0; i < 3; i++){
            data_holder.accel_cal[i] = data_holder.accel[i] - data_holder.accel_offset[i];
            data_holder.gyro_cal[i] = data_holder.gyro[i] - data_holder.gyro_offset[i];
      }
}

/**
 * \brief Performs temperature compensation on calibrated accelerometer and gyroscope readings
 *        using typical board-level temperature drift specifications from the datasheet:
 *        - Accel Zero-G drift: 0.15 mg/°C (0.0014715 m/s^2/°C)
 *        - Gyro Zero-Rate drift: 0.015 dps/°C (0.0002618 rad/s/°C)
 */
void ICM42607::temp_correct() {
      // Nominal datasheet typical offset drift coefficients
      constexpr float ACCEL_TEMP_COEFF = 0.15e-3f * GRAVITY_ICM;    // ~0.0014715 m/s^2 per deg C
      constexpr float GYRO_TEMP_COEFF  = 0.015f * DEG2RAD_ICM;   // ~0.0002618 rad/s per deg C

      // Calculate deviation from nominal 25 deg C room temperature reference
      float temp_delta = data_holder.temp - 25.0f;

      for (int i = 0; i < 3; i++) {
            data_holder.accel_cal[i] -= (temp_delta * ACCEL_TEMP_COEFF);
            data_holder.gyro_cal[i]  -= (temp_delta * GYRO_TEMP_COEFF);
      }
}

//Below is the calibration block

/**
 * \brief Function to calibrate the accelerometer. This should be called during first time start up with calibration values stored in the on board SPI flash memory
 * \return success of calibration
 */
bool ICM42607::accel_calib(int num_samples){

      data_holder.accel_offset[0] = 0;
      data_holder.accel_offset[1] = 0;
      data_holder.accel_offset[2] = 0;//Zero all offsets to generate new set

      float ax_avg = 0;
      float ay_avg = 0;
      float az_avg = 0;//Accumilated average

      int sample_count = 0;
      while (sample_count < num_samples){

            if (new_data_ready) {
                  sample_count ++;

                  new_data_ready = false;
                  single_read();
                  //Serial.printf("ICM Data: %.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n",data_holder.accel[0],data_holder.accel[1],data_holder.accel[2],data_holder.gyro[0],data_holder.gyro[1],data_holder.gyro[2]);
                  ax_avg += data_holder.accel[0];
                  ay_avg += data_holder.accel[1];
                  az_avg += data_holder.accel[2];
            }
            
      }
      //Now that the requsite samples are collected, average the offsets
      
      data_holder.accel_offset[0] = ax_avg/((float)sample_count);
      data_holder.accel_offset[1] = ay_avg/((float)sample_count);
      data_holder.accel_offset[2] = az_avg/((float)sample_count) - 9.81; //Subtract the gravity vector

      Serial.printf("[ICM] IMU accel offsets: x: %.4f, y: %.4f, z: %.4f\n", data_holder.accel_offset[0],data_holder.accel_offset[1],data_holder.accel_offset[2]);    

      Serial.println("[ICM] Accel calibration complete!");

      return 1;
}

/**
 * \brief Function to calibrate the gyroscope. This should be called during first time start up with calibration values stored in the on board SPI flash memory
 * \return success of calibration
 */
bool ICM42607::gyro_calib(int num_samples){
      data_holder.gyro_offset[0] = 0;
      data_holder.gyro_offset[1] = 0;
      data_holder.gyro_offset[2] = 0;//Zero all offsets to generate new set

      float gx_avg = 0;
      float gy_avg = 0;
      float gz_avg = 0;//Accumilated average

      int sample_count = 0;
      while (sample_count < num_samples){
            if (new_data_ready) {
                  sample_count ++;

                  new_data_ready = false;
                  single_read();
                  //Serial.printf("ICM Data: %.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n",data_holder.accel[0],data_holder.accel[1],data_holder.accel[2],data_holder.gyro[0],data_holder.gyro[1],data_holder.gyro[2]);
                  gx_avg += data_holder.gyro[0];
                  gy_avg += data_holder.gyro[1];
                  gz_avg += data_holder.gyro[2];
            }
            
      }
      //Now that the requsite samples are collected, average the offsets
      
      data_holder.gyro_offset[0] = gx_avg/((float)sample_count);
      data_holder.gyro_offset[1] = gy_avg/((float)sample_count);
      data_holder.gyro_offset[2] = gz_avg/((float)sample_count);

      Serial.printf("[ICM] gyro offsets: x: %.4f, y: %.4f, z: %.4f\n", data_holder.gyro_offset[0],data_holder.gyro_offset[1],data_holder.gyro_offset[2]);    

      Serial.println("[ICM] Gyro calibration complete!");

      return 1;
}