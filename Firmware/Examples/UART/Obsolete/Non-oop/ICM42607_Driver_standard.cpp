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
volatile bool dma_in_progress_ICM = false;
volatile bool new_data_ready_ICM = false;
bool IMU_first_read = false;

//Data holding structure
ICM_Data_t ICM_Data_Holder = {};

//Scaling constants for accel and gyro
constexpr float ACCEL_LSB_PER_G[4] = {2048.0, 4096.0, 8192.0, 16384.0};
constexpr float GYRO_LSB_PER_DPS[4] = {16.384, 32.768, 65.536, 131.072};

//RTOS Task handle
extern TaskHandle_t SensorTaskHandle; //Define this task handle where the sensor data is processed

// FUNCTIONS

//First two functions are a pair of quick ISR functions for data ready & DMA complete

//ISR for data ready interrupt from chip
void IRAM_ATTR ICM_ISR_dataReady() {
      if (!dma_in_progress_ICM) {
            // Queue the pre-armed DMA transaction with zero tick delay
            if (spi_device_queue_trans(SPI_DMA_ICM.handle, &SPI_DMA_ICM.trans, 0) == ESP_OK) {
                  dma_in_progress_ICM = true;
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
void IRAM_ATTR ICM_ISR_DMAcomplete_callback(spi_transaction_t *trans) {
      new_data_ready_ICM = true; // Raise flag for buffer full
      if (using_RTOS){
            // Wake up the RTOS task
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (SensorTaskHandle != NULL) {
                  vTaskNotifyGiveFromISR(SensorTaskHandle, &xHigherPriorityTaskWoken);//This function checks if sensor task has higher priority than current task during isr firing
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
 * \param ICM_Config configuration struct, consult ICM42607_Driver.h for details on config fields
 * \return boolean value true meaning successfully initialized and false failed
 */
bool ICM_init_chip(ICM_Config_t ICM_Config) {

      ICM_Data_Holder = {};//Empty out any holder value during initialization

      //First setup the SPI bus

      // Initialize the SPI bus
      if (!SPI_Bus_Init(PIN_MOSI_ICM, PIN_MISO_ICM, PIN_SCLK_ICM)) {
            Serial.println("[ICM-ERROR] SPI host initialization failed!");
            return 0;
      }

      // Add the specific SPI device
      if (!SPI_Add_Device(PIN_CS_ICM, ICM_ISR_DMAcomplete_callback, SPI_DMA_ICM.handle)) {
            Serial.println("[ICM-ERROR] Could not add ICM Device!");
            return 0;
      }

      // Initialize the SPI bus
      if (!SPI_Arm_DMA_Channel(ICM_BURST_LEN, (DATA_START_ICM  | SPI_READ_FLAG), SPI_DMA_ICM)) {
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

      //First, We will use bitwise operations to construct the configuration for sample rate, bandwidth, and range of both accel and gyro
      uint8_t accel_ODR = 13 - ICM_Config.outputRate; //The accel odr ranges from 5 (1600 Hz) to 12 (12.5Hz) on the ACCEL_CONFIG0 register.
      uint8_t gyro_ODR  = 13 - ICM_Config.outputRate; //same as accel

      uint8_t accel_range = (4 - ICM_Config.accel_range); //Accel full scale ranges from 0 (±16g) to 3 (±2g) left shifted 5 bits on ACCEL_CONFIG0
      ICM_Data.accel_conversion = ACCEL_LSB_PER_G[accel_range];
      uint8_t gyro_range = (4 - ICM_Config.gyro_range); //Gyro full scale ranges from 0 (±2000dps) to 3 (±250dps) left shifted 5 bits on GYRO_CONFIG0
      ICM_Data.gyro_conversion = GYRO_LSB_PER_DPS[gyro_range];

      uint8_t accel_bw = 8 - ICM_Config.accel_bw; //Bandwidth goes from 1 (180Hz) to 7 (16Hz) with 0 being no filter on Accel_CONFIG1
      uint8_t gyro_bw = 8 - ICM_Config.gyro_bw; //same as accel

      //Construct config0 for data rate and full scale range
      ICM_write_reg(ACCEL_CONFIG0_ICM,  (accel_ODR | (accel_range<<5)));
      ICM_write_reg(GYRO_CONFIG0_ICM, (gyro_ODR | (gyro_range<<5)));
      //Construct config1 for bandwidth
      ICM_write_reg(ACCEL_CONFIG1_ICM,  accel_bw);
      ICM_write_reg(GYRO_CONFIG1_ICM,  gyro_bw);

      //Set temperature bandwidith
      ICM_write_reg(TEMP_CONFIG0,  0x04);//16 hz for filtering temperature

      //Set up the interrupts
      //Set in pulse mode, push pull, active high, 0b00 011011 0x1B
      ICM_write_reg(INT_CONFIG_ICM, 0x1B);
      
      //Set int 2, 0b00001000, data ready
      //ICM_write_reg(INT_SOURCE3_ICM, 0x08);
      
      //Set int 1, 0b00001000, data ready
      ICM_write_reg(INT_SOURCE0_ICM, 0x08);

      //Set power mode
      // 0b0000 1111, low noise mode for both accel and gyro
      ICM_write_reg(PWR_MGMT0_ICM, 0x0F);
      
      //Check if we are communicating with the right chip
      uint8_t check_addr = ICM_read_reg(WHO_AM_I_ICM);
      if (check_addr != 0x60){
            Serial.printf("[ICM-ERROR] Wrong address on ICM! Address found to be %x, address we are looking for is 0x60\n", check_addr);
            return false; //Checking who am I failed, IMU not initialized
      }
      else{
            Serial.printf("[ICM] Correct address on ICM, found to be %x.\n",check_addr);
      }

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

      int16_t accel_raw[3];
      int16_t gyro_raw[3];
      
      accel_raw[0] = (int16_t)((SPI_DMA_ICM.rx_buffer[3]  << 8) | SPI_DMA_ICM.rx_buffer[4]);
      accel_raw[1] = (int16_t)((SPI_DMA_ICM.rx_buffer[5]  << 8) | SPI_DMA_ICM.rx_buffer[6]);
      accel_raw[2] = (int16_t)((SPI_DMA_ICM.rx_buffer[7]  << 8) | SPI_DMA_ICM.rx_buffer[8]);
      
      gyro_raw[0] = (int16_t)((SPI_DMA_ICM.rx_buffer[9]  << 8) | SPI_DMA_ICM.rx_buffer[10]);
      gyro_raw[1] = (int16_t)((SPI_DMA_ICM.rx_buffer[11] << 8) | SPI_DMA_ICM.rx_buffer[12]);
      gyro_raw[2] = (int16_t)((SPI_DMA_ICM.rx_buffer[13] << 8) | SPI_DMA_ICM.rx_buffer[14]);

      
      //Convert accel and gyro values with scaling

      for (int i = 0; i < 3; i++){
            ICM_Data_Holder.accel[i] = accel_raw[i] / ICM_Data_Holder.accel_conversion * GRAV;
            ICM_Data_Holder.gyro[i] = gyro_raw[i] / ICM_Data_Holder.gyro_conversion * DEG2RAD;
      }
      
      //Apply calibration
      ICM_apply_calibration();

      return true;
}

/**
 * \brief Function to apply calibration offsets to accel and gyro values
 * \return nothing
 */
void ICM_apply_calibration(){
      for (int i = 0; i < 3; i++){
            ICM_Data_Holder.accel_cal[i] = ICM_Data_Holder.accel[i] - ICM_Data_Holder.accel_offset[i];
            ICM_Data_Holder.gyro_cal[i] = ICM_Data_Holder.gyro[i] - ICM_Data_Holder.gyro_offset[i];
      }
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
      ICM_Data_Holder.az_offset = ax_avg/((float)sample_count) - 9.81; //Subtract the gravity vector

      Serial.printf("[ICM] IMU accel offsets: x: %.4f, y: %.4f, z: %.4f\n", ICM_Data_Holder.ax_offset,ICM_Data_Holder.ay_offset,ICM_Data_Holder.az_offset);    

      Serial.println("[ICM] Accel calibration complete!");

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
      ICM_Data_Holder.gz_offset = gx_avg/((float)sample_count);

      Serial.printf("[ICM] gyro offsets: x: %.4f, y: %.4f, z: %.4f\n", ICM_Data_Holder.gx_offset,ICM_Data_Holder.gy_offset,ICM_Data_Holder.gz_offset);    

      Serial.println("[ICM] Gyro calibration complete!");

      return 1;

}

