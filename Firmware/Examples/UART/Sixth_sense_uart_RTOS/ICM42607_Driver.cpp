/**
 * \file      ICM42607_Driver.cpp
 * \brief     Library for interfacing with ICM42607.
 *
 * \authors   Chase Sun
 */

// INCLUDES
#include "ICM42607_Driver.h"


// Initialze the global variables declared in the header
//Variables used for SPI communication
SPI_DMA_Channel SPI_DMA_ICM = {};

//Flags for data ready
volatile bool dma_in_progress = false;
volatile bool new_data_ready_ICM = false;
bool IMU_first_read = false;

//Data holding structure
ICM_Data_t ICM_Data_Holder = {};

//RTOS Task handle
extern TaskHandle_t SensorTaskHandle; //Define this task handle where the sensor data is processed

// FUNCTIONS

//First two functions are a pair of quick ISR functions for data ready & DMA complete

//ISR for data ready interrupt from chip
void IRAM_ATTR IMU_ISR_dataReady() {
      if (!dma_in_progress_ICM) {
            // Queue the pre-armed DMA transaction with zero tick delay
            if (spi_device_queue_trans(SPI_DMA_ICM.handle, &SPI_DMA_ICM.trans, 0) == ESP_OK) {
                  dma_in_progress_ICM = true;
            }
      }
}

//Call back function when DMA completes transfer and for triggering data processing & communication
void IRAM_ATTR IMU_ISR_DMAcomplete_callback(spi_transaction_t *trans) {
      new_data_ready_ICM = true; // Raise flag for buffer full
      
      // Wake up the RTOS task
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      if (SensorTaskHandle != NULL) {
          vTaskNotifyGiveFromISR(SensorTaskHandle, &xHigherPriorityTaskWoken);//This function checks if sensor task has higher priority than current task during isr firing
          //If that is the case (should be in almost all cases as the sensor task is very high priority), the function will set the boolean to pdTrue
          //Then, the ISR will exit and the sensor reading task is immediately executed before the current task is finished executing.
          portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
}


/**
 * \brief Tool to write SPI command to a register
 *
 * \param reg //The SPI register to write to
 * \param data //The data to write to
 * \return
 */
void ICM_write_reg(uint8_t reg, uint8_t data) {
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 16; //Two bytes
    t.tx_data[0] = reg;
    t.tx_data[1] = data;
    spi_device_polling_transmit(SPI_DMA_ICM.handle, &t);
}


/**
 * \brief Tool to read SPI register. This only reads 1 register for checking config. Do not use this to read actual Accel/Gyro/Mag data as its too slow
 *
 * \param reg //The SPI register to write to
 * \return
 */
uint8_t ICM_read_reg(uint8_t reg) {
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 16;
    t.tx_data[0] = reg | SPI_READ_FLAG;
    t.tx_data[1] = 0x00; //send 0s so the read is valid
    spi_device_polling_transmit(SPI_DMA_ICM.handle, &t);
    return t.rx_data[1];
}


/**
 * \brief Function to initialize a single ICM42607
 *
 * \param outputRate the rate setting for the ICM output.
 *                      1. 100hz
 *                      2. 200hz (default)
 *                      3. 400hz
 *                      4. 800hz
 *                      5. 1600hz
 * \return boolean value true meaning successfully initialized and false failed
 */
bool ICM_init_chip(uint8_t outputRate) {

      ICM_Data_Holder = {};//Empty out any holder value during initialization

      //First setup the SPI bus

      // Initialize the SPI bus
      if (!SPI_Bus_Init(PIN_MOSI_ICM, PIN_MISO_ICM, PIN_SCLK_ICM)) {
            Serial.println("SPI host initialization failed!");
            return 0;
      }

      // Add the specific SPI device
      if (!SPI_Add_Device(PIN_CS_ICM, IMU_ISR_DMAcomplete_callback, SPI_DMA_ICM.handle)) {
            Serial.println("Could not add ICM Device!");
            return 0;
      }

      // Initialize the SPI bus
      if (!SPI_Arm_DMA_Channel(ICM_BURST_LEN, (DATA_START_ICM  | SPI_READ_FLAG), SPI_DMA_ICM)) {
            Serial.println("DMA setup for ICM failed!");
            return 0;
      }

      //For configuration, we do 7 steps
      //1. set filter bandwidth
      //2. Set divider for reporting frequency - 400 hz default, controls the low level control loop speed if IMU mode is enabled
      //3. Set accel range - 8g default
      //4. Set gyro range - 500deg default
      //5. Set interrupts. For non-back imus, enable int 2. for back imu enable 1 and 2
      //6. Set PWR_MGMT0 register to configure clock, gyro and accel mode
      //7. Check if the IMU id is what we expect - this checks if there is a valid connection to the IMU after all configuration
      //8. Set the valid flag in IMU data struct to 1 so that its data can be streamed.

      //First, set sample rate, range, and bandwidth
      if (outputRate == 5){
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x45); // 1600
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x25);
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x01); //Bandwidth 180hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x01);
            ICM_Data_Holder.frequency = 1600;
      }
      else if(outputRate == 4){
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x46); //800
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x26);
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x01); //Bandwidth 180hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x01);
            ICM_Data_Holder.frequency = 800;
      }
      else if(outputRate == 3){
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x47); // 400
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x27);
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x01); //Bandwidth 34hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x05);
            ICM_Data_Holder.frequency = 400;
      }
      else if(outputRate == 2){//Microstrain equivalent settings for comparison
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x28);//1000dps, 200hz, matching microstrain
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x28);//8g, 200hz, matching microstrain
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x02);//half of SR, 121hz, microstrain 100hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x03);//half of SR, 73hz, microstrain 100hz
            ICM_Data_Holder.frequency = 200;
      }
      else if(outputRate == 1){
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x49); //100
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x29);
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x04); //Bandwidth 16hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x04);
            ICM_Data_Holder.frequency = 100;
      }
      else{
            Serial.println("Report frequency can be only (5) 1600 / (4) 800 / (3) 400 / (2) 200 / (1) 100 hz.");
            return false;
      }

      
      //Set up the interrupts
      //Set in pulse mode, push pull, active high, 0b00 011011 0x1B
      ICM_write_reg(INT_CONFIG_ICM, 0x1B);
      
      //Set int 2, 0b00001000, data ready
      //ICM_write_reg(INT_SOURCE3_ICM, 0x08);
      
      //Set int 1, 0b00001000, data ready
      ICM_write_reg(INT_SOURCE0_ICM, 0x08);

      //Set power mode
      // 0b0000 1111
      ICM_write_reg(PWR_MGMT0_ICM, 0x0F);
      
      //Check if we are communicating with the right chip
      uint8_t check_addr = ICM_read_reg(WHO_AM_I_ICM);
      if (check_addr != 0x60){
            Serial.printf("Wrong address on ICM! Address found to be %x, address we are looking for is 0x60\n", check_addr);
            return false; //Checking who am I failed, IMU not initialized
      }
      else{
            Serial.printf("Correct address on ICM, found to be %x.\n",check_addr);
      }

      //Last step: set the kalman filter parameters (defaults used)
      ICM_Data_Holder.kalmanRoll.setQangle(0.001);
      ICM_Data_Holder.kalmanRoll.setQbias(0.003);
      ICM_Data_Holder.kalmanRoll.setRmeasure(0.03);

      ICM_Data_Holder.kalmanPitch.setQangle(0.001);
      ICM_Data_Holder.kalmanPitch.setQbias(0.003);
      ICM_Data_Holder.kalmanPitch.setRmeasure(0.03);

      return true;
}


/**
 * \brief Function to process DMA output from interrupt. At this point, the data is loaded into the dma_rx_ICM buffer and is ready to be read
 * \return boolean whether read was successful
 */
 bool ICM_single_read(){

      spi_transaction_t *r_trans;

      if (spi_device_get_trans_result(SPI_DMA_ICM.handle, &r_trans, 0) != ESP_OK) {
            return false;
      }

      dma_in_progress_ICM = false;
      
      // Make the read

      // Unpack raw 16-bit values
      int16_t temp_raw = (int16_t)((SPI_DMA_ICM.rx_buffer[1]  << 8) | SPI_DMA_ICM.rx_buffer[2]);

      ICM_Data_Holder.temp  = ((float)temp_raw / 128.0f) + 25.0f;

      int16_t ax_raw = (int16_t)((SPI_DMA_ICM.rx_buffer[3]  << 8) | SPI_DMA_ICM.rx_buffer[4]);
      int16_t ay_raw = (int16_t)((SPI_DMA_ICM.rx_buffer[5]  << 8) | SPI_DMA_ICM.rx_buffer[6]);
      int16_t az_raw = (int16_t)((SPI_DMA_ICM.rx_buffer[7]  << 8) | SPI_DMA_ICM.rx_buffer[8]);
      
      int16_t gx_raw = (int16_t)((SPI_DMA_ICM.rx_buffer[9]  << 8) | SPI_DMA_ICM.rx_buffer[10]);
      int16_t gy_raw = (int16_t)((SPI_DMA_ICM.rx_buffer[11] << 8) | SPI_DMA_ICM.rx_buffer[12]);
      int16_t gz_raw = (int16_t)((SPI_DMA_ICM.rx_buffer[13] << 8) | SPI_DMA_ICM.rx_buffer[14]);

      // --- Conversion constants (match your configured ranges) ---
      // Accel LSB per g: ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048
      const float ACCEL_LSB_PER_G = 4096.0f;     // for ±8g
      const float GYRO_LSB_PER_DPS = 32.8f;      // for ±1000 dps
      const float G = 9.80665f;

      
      //Make distinction in calibration mode
      //if (calibrationMode){
      if (1 == 0){
            //If we are calibrating - should not be the case during normal operation
            //We will accumilate the measurements in the calibration field to be averaged later
            ICM_Data_Holder.gx = gx_raw / GYRO_LSB_PER_DPS;
            ICM_Data_Holder.gy = gy_raw / GYRO_LSB_PER_DPS;
            ICM_Data_Holder.gz = gz_raw / GYRO_LSB_PER_DPS;

            ICM_Data_Holder.ax = (ax_raw / ACCEL_LSB_PER_G) * G;
            ICM_Data_Holder.ay = (ay_raw / ACCEL_LSB_PER_G) * G;
            ICM_Data_Holder.az = (az_raw / ACCEL_LSB_PER_G) * G;

            ICM_Data_Holder.gx_offset += ICM_Data_Holder.gx;
            ICM_Data_Holder.gy_offset += ICM_Data_Holder.gy;
            ICM_Data_Holder.gz_offset += ICM_Data_Holder.gz;
      }
      else{
            //Not in calibration mode, we either have valid constants or they are zero, subtract regardless
            ICM_Data_Holder.gx = gx_raw / GYRO_LSB_PER_DPS - ICM_Data_Holder.gx_offset;
            ICM_Data_Holder.gy = gy_raw / GYRO_LSB_PER_DPS - ICM_Data_Holder.gy_offset;
            ICM_Data_Holder.gz = gz_raw / GYRO_LSB_PER_DPS - ICM_Data_Holder.gz_offset;

            ICM_Data_Holder.ax = (ax_raw / ACCEL_LSB_PER_G) * G - ICM_Data_Holder.ax_offset;//Apply hardcoded factory offset
            ICM_Data_Holder.ay = (ay_raw / ACCEL_LSB_PER_G) * G - ICM_Data_Holder.ay_offset;
            ICM_Data_Holder.az = (az_raw / ACCEL_LSB_PER_G) * G - ICM_Data_Holder.az_offset;
      }



      return true;
}




/**
 * \brief Function to perform Kalman filtering of a single IMU. Populates the Euler angle fields of the IMU struct
 * the function will only perform kalman filtering if IMU is active
 * \param dt the time elapsed from the last read for gyro integration
 * \param mode mode selector for flat (microstrain default) or upright (better for yaw sensing)
 * \return nothing
 */
 void ICM_Kalman_fusion(float dt, int mode){
      //default mode 0 - flat mode, xyz axis stay in tact
      
      float ax = ICM_Data_Holder.ax;
      float ay = ICM_Data_Holder.ay;
      float az = ICM_Data_Holder.az;

      float gx = ICM_Data_Holder.gx;
      float gy = ICM_Data_Holder.gy;
      float gz = ICM_Data_Holder.gz;


      if (mode == 1){//Upright mode, y stays the same, raw z maps to x, x maps to negative z
            ax = ICM_Data_Holder.az;
            az = -1.0*ICM_Data_Holder.ax;

            gx = ICM_Data_Holder.gz;
            gz = -1.0*ICM_Data_Holder.gx;
      }

      float rollAcc  = atan2(ay, az) * RAD_TO_DEG;
      float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

      if(IMU_first_read){
            ICM_Data_Holder.kalmanRoll.setAngle(rollAcc);
            ICM_Data_Holder.kalmanPitch.setAngle(pitchAcc);
            ICM_Data_Holder.roll = rollAcc;
            ICM_Data_Holder.pitch = pitchAcc; //If first reading, the roll and pitch will be purely based on accelerometer.
            //The first read boolean can also be held true to disable the kalman filter for debugging

            //Compute yaw using accel angle readings
            return;
      }
      //Now that first read is over, we actually engage the kalman filter
      
      //Compute sensor fusion of roll and pitch first as we need them for 
      ICM_Data_Holder.roll = ICM_Data_Holder.kalmanRoll.getAngle(rollAcc, gx, dt);

      ICM_Data_Holder.pitch = ICM_Data_Holder.kalmanPitch.getAngle(pitchAcc, gy, dt);

 }      
      

/**
 * \brief Function to compute a moving average
 * \return the averaged value
 */
float moving_average(float *buffer, float new_val, int &ma_index){
      buffer[ma_index] = new_val;
      // Add new value

      // Update index (circular)
      ma_index = (ma_index + 1) % Moving_average_windowSize;
      float sum = 0;
      for (size_t i = 0; i < Moving_average_windowSize; ++i) {
            sum += buffer[i];
      }
      // Return average
      return sum / (float)Moving_average_windowSize;
}

//Below is the calibration block

/**
 * \brief Function to calibrate the accelerometer. This should be called during first time start up with calibration values stored in the on board SPI flash memory
 * \return success of calibration
 */
bool ICM_accel_calib(int num_samples){

      ICM_Data_Holder.ax_offset = 0;
      ICM_Data_Holder.ay_offset = 0;
      ICM_Data_Holder.az_offset = 0;//Zero all offsets to generate new set

      float ax_avg = 0;
      float ay_avg = 0;
      float az_avg = 0;//Accumilated average

      int sample_count = 0;
      while (sample_count < num_samples){

            if (new_data_ready_ICM) {
                  sample_count ++;

                  new_data_ready_ICM = false;
                  ICM_single_read();
                  //Serial.printf("ICM Data: %.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n",ICM_Data_Holder.ax,ICM_Data_Holder.ay,ICM_Data_Holder.az,ICM_Data_Holder.gx,ICM_Data_Holder.gy,ICM_Data_Holder.gz);
                  ax_avg += ICM_Data_Holder.ax;
                  ay_avg += ICM_Data_Holder.ay;
                  az_avg += ICM_Data_Holder.az;
            }
            
      }
      //Now that the requsite samples are collected, average the offsets
      
      ICM_Data_Holder.ax_offset = ax_avg/((float)sample_count);
      ICM_Data_Holder.ay_offset = ay_avg/((float)sample_count);
      ICM_Data_Holder.az_offset = ax_avg/((float)sample_count);//Subtracting the gravity vector

      Serial.printf("IMU accel offsets: x: %.4f, y: %.4f, z: %.4f\n", ICM_Data_Holder.ax_offset,ICM_Data_Holder.ay_offset,ICM_Data_Holder.az_offset);    

      Serial.println("Accel calibration complete!");

      return 1;

}

/**
 * \brief Function to calibrate the gyroscope. This should be called during first time start up with calibration values stored in the on board SPI flash memory
 * \return success of calibration
 */
bool ICM_gyro_calib(int num_samples){
      ICM_Data_Holder.gx_offset = 0;
      ICM_Data_Holder.gy_offset = 0;
      ICM_Data_Holder.gz_offset = 0;//Zero all offsets to generate new set

      float gx_avg = 0;
      float gy_avg = 0;
      float gz_avg = 0;//Accumilated average

      int sample_count = 0;
      while (sample_count < num_samples){

            if (new_data_ready_ICM) {
                  sample_count ++;

                  new_data_ready_ICM = false;
                  ICM_single_read();
                  //Serial.printf("ICM Data: %.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n",ICM_Data_Holder.ax,ICM_Data_Holder.ay,ICM_Data_Holder.az,ICM_Data_Holder.gx,ICM_Data_Holder.gy,ICM_Data_Holder.gz);
                  gx_avg += ICM_Data_Holder.gx;
                  gy_avg += ICM_Data_Holder.gy;
                  gz_avg += ICM_Data_Holder.gz;
            }
            
      }
      //Now that the requsite samples are collected, average the offsets
      
      ICM_Data_Holder.gx_offset = gx_avg/((float)sample_count);
      ICM_Data_Holder.gy_offset = gy_avg/((float)sample_count);
      ICM_Data_Holder.gz_offset = gx_avg/((float)sample_count);//Subtracting the gravity vector

      Serial.printf("IMU gyro offsets: x: %.4f, y: %.4f, z: %.4f\n", ICM_Data_Holder.gx_offset,ICM_Data_Holder.gy_offset,ICM_Data_Holder.gz_offset);    

      Serial.println("Gyro calibration complete!");

      return 1;

}

