/**
 * \file      ICM42607_Driver.cpp
 * \brief       Library for interfacing with ICM42607.
 *
 * \authors     Chase Sun
 * \bug
 */

// INCLUDES
#include "ICM42607_Driver.h"

// Initialze the global variables declared in the header
spi_device_handle_t spi_ICM = NULL;
uint8_t *dma_tx_ICM = nullptr;
uint8_t *dma_rx_ICM = nullptr;
spi_transaction_t dma_trans_ICM;
volatile bool dma_in_progress = false;
volatile bool new_data_ready = false;
bool IMU_first_read = false;
ICM_Data ICM_Data_Holder;


// FUNCTIONS

//First two functions are a pair of quick ISR functions for data ready & DMA complete

//ISR for data ready interrupt from chip
void IRAM_ATTR IMU_ISR_dataReady() {
      if (!dma_in_progress) {
            // Queue the pre-armed DMA transaction with zero tick delay
            if (spi_device_queue_trans(spi_ICM, &dma_trans_ICM, 0) == ESP_OK) {
                  dma_in_progress = true;
            }
      }
}

//ISR for DMA SPI Transaction complete
void IRAM_ATTR IMU_ISR_DMAcomplete_callback(spi_transaction_t *trans) {
      new_data_ready = true; //Raise flag for buffer full
}

/**
 * \brief Tool to config SPI BUS
 *
 * \return
 */
 bool ICM_SPI_config(){
      spi_bus_config_t buscfg = {};
      buscfg.mosi_io_num = PIN_MOSI;
      buscfg.miso_io_num = PIN_MISO;
      buscfg.sclk_io_num = PIN_SCLK;
      buscfg.quadwp_io_num = -1;
      buscfg.quadhd_io_num = -1;
      buscfg.max_transfer_sz = 4096;
      esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
      if (err != ESP_OK) return false;

      spi_device_interface_config_t devcfg = {};

      devcfg.clock_speed_hz = 10 * 1000 * 1000; // 10 MHz
      devcfg.mode = 0;                    // SPI Mode 0
      devcfg.spics_io_num = PIN_CS;
      devcfg.queue_size = 2;                // Allows for 2 transactions at a time to give a bit of space
      devcfg.post_cb = IMU_ISR_DMAcomplete_callback;        // register the callback function
      err = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_ICM);
      return (err == ESP_OK);
}

/**
 * \brief Set up DMA buffer for ICM
 *
 * \return
 */
bool ICM_DMA_config() {
      // 1. Allocate buffers in internal, DMA-accessible SRAM
      dma_tx_ICM = (uint8_t *)heap_caps_malloc(ICM_BURST_LEN, MALLOC_CAP_DMA);//The TX buffer matches the RX buffer to send 0s during read for full duplex operation
      dma_rx_ICM = (uint8_t *)heap_caps_malloc(ICM_BURST_LEN, MALLOC_CAP_DMA);

      if (!dma_tx_ICM || !dma_rx_ICM) {
            return false; //Check if the heap memory is correctly assigned
      }

      // 2. Pre-arm the TX buffer with the register address command
      memset(dma_tx_ICM, 0, ICM_BURST_LEN);
      dma_tx_ICM[0] = DATA_START_ICM | SPI_READ_FLAG; //Address | flag + zeros the rest of the way

      // 3. Pre-arm the reusable transaction manifest
      memset(&dma_trans_ICM, 0, sizeof(spi_transaction_t)); 
      dma_trans_ICM.length = ICM_BURST_LEN * 8; // Bit count (8 * 8 = 64 bits), SPI DMA driver uses bits to write/read exactly 8 bytes of data
      dma_trans_ICM.tx_buffer = dma_tx_ICM;
      dma_trans_ICM.rx_buffer = dma_rx_ICM;

      return true;
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
    spi_device_polling_transmit(spi_ICM, &t);
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
    spi_device_polling_transmit(spi_ICM, &t);
    return t.rx_data[1];
}


/**
 * \brief Function to initialize a single ICM42607
 *
 * \param reportFreqnency the frequency to report data. If ran in IMU mode, this determines the frequency of the control loop
 * \return boolean value true meaning successfully initialized and false failed
 */
bool ICM_init_chip(int reportFrequency) {

      ICM_Data_Holder = {};//Empty out any holder value during initialization
      
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
      if (reportFrequency == 1600){
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x45); // 1600
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x25);
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x01); //Bandwidth 180hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x01);
      }
      else if(reportFrequency == 800){
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x46); //800
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x26);
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x01); //Bandwidth 180hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x01);
      }
      else if(reportFrequency == 400){
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x47); // 400
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x27);
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x01); //Bandwidth 34hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x05);
      }
      else if(reportFrequency == 200){//Microstrain equivalent settings for comparison
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x28);//1000dps, 200hz, matching microstrain
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x28);//8g, 200hz, matching microstrain
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x02);//half of SR, 121hz, microstrain 100hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x03);//half of SR, 73hz, microstrain 100hz
      }
      else if(reportFrequency == 100){
            ICM_write_reg(GYRO_CONFIG0_ICM,  0x49); //100
            ICM_write_reg(ACCEL_CONFIG0_ICM, 0x29);
            ICM_write_reg(GYRO_CONFIG1_ICM,  0x04); //Bandwidth 16hz
            ICM_write_reg(ACCEL_CONFIG1_ICM, 0x04);
      }
      else{
            Serial.println("Report frequency can be only 1600 / 800 / 400 / 200 / 100 hz.");
            return false;
      }
      ICM_Data_Holder.frequency = reportFrequency;
      
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
 * \brief Function to calibrate the gyroscope of all sensors simutaniously. The function populates the gx_offset, gy_offset, and gz_offset data fields in the imu struct
 * takes advantage of the interrupt and ISR to time reads. Calibrates all online sensors at once
 * \param num_samples the number of samples to average. Default 400
 * \param interrupt the ISR boolean used as a clock
 * \return nothing
 */ /**
 void ICM_calibration(int num_samples, volatile bool& interrupt){
      int sample_count = 0;
      while (sample_count < num_samples){
            if (interrupt){
                  //Interrupt triggered, count the loop
                  interrupt = false;//reset int
                  sample_count ++;
                  read_all_MS5K(true);//read all available imus, calibration mode is set to true
                  //The offset terms should accumilate
            }
      }
      //Now that the requsite samples are collected, average the offsets
      for (int i = 0; i < (int)NUM_MS5K; i++){
            if (MS5K_Data_Holder[i].valid_flag == true){
                  MS5K_Data_Holder[i].gx_offset = MS5K_Data_Holder[i].gx_offset/((float)sample_count);
                  MS5K_Data_Holder[i].gy_offset = MS5K_Data_Holder[i].gy_offset/((float)sample_count);
                  MS5K_Data_Holder[i].gz_offset = MS5K_Data_Holder[i].gz_offset/((float)sample_count);
                  Serial.printf("IMU %d gyro offsets: x: %.4f, y: %.4f, z: %.4f\n", i, MS5K_Data_Holder[i].gx_offset,MS5K_Data_Holder[i].gy_offset,MS5K_Data_Holder[i].gz_offset);    
            }
      }
      Serial.println("Calibration complete!");
 }


/**
 * \brief Function to process DMA output from interrupt. At this point, the data is loaded into the dma_rx_ICM buffer and is ready to be read
 * \return boolean whether read was successful
 */
 bool ICM_single_read(){

      spi_transaction_t *r_trans;
      if (spi_device_get_trans_result(spi_ICM, &r_trans, 0) == ESP_OK) {
            dma_in_progress = false;
            return false;
      }
      

      // Make the read


      // Unpack raw 16-bit values
      int16_t temp_raw = (int16_t)((dma_rx_ICM[1]  << 8) | dma_rx_ICM[2]);

      ICM_Data_Holder.temp  = ((float)temp_raw / 128.0f) + 25.0f;

      int16_t ax_raw = (int16_t)((dma_rx_ICM[3]  << 8) | dma_rx_ICM[4]);
      int16_t ay_raw = (int16_t)((dma_rx_ICM[5]  << 8) | dma_rx_ICM[6]);
      int16_t az_raw = (int16_t)((dma_rx_ICM[7]  << 8) | dma_rx_ICM[8]);
      
      int16_t gx_raw = (int16_t)((dma_rx_ICM[9]  << 8) | dma_rx_ICM[10]);
      int16_t gy_raw = (int16_t)((dma_rx_ICM[11] << 8) | dma_rx_ICM[12]);
      int16_t gz_raw = (int16_t)((dma_rx_ICM[13] << 8) | dma_rx_ICM[14]);

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
 * \brief Function to factory calibrate accelerometer. Should only be called in a standalone script and should not be ran during regular operation
 * It generates 
 * \param IMU_ID id of the IMU to be calibrated. Only do this one at a time as IMU needs to be perfectly flat
 * \param num_samples the number of samples to average. Default 400
 * \param interrupt the ISR boolean used as a clock
 * \return nothing
 *//**
 void ICM_factory_accel_calibration(int IMU_ID, int num_samples, volatile bool& interrupt){
      //First check if IMU_ID is valid
      if (IMU_ID < 0 || IMU_ID > ((int)NUM_MS5K-1)){
            return;//Just return fail read if the ID is not valid
      }
      //First check if IMU is actually initialized
      if (ICM_Data_Holder.valid_flag == false){
            return;//Just return fail read if the IMU is not being used
      }
      
      Serial.printf("IMU %d accel calibration beginning in 3 seconds. Keep it flat and face up\n",IMU_ID);
      delay(1000);
      Serial.printf("IMU %d accel calibration beginning in 2 seconds. Keep it flat and face up\n",IMU_ID);
      delay(1000);
      Serial.printf("IMU %d accel calibration beginning in 1 seconds. Keep it flat and face up\n",IMU_ID);
      delay(1000);
      Serial.printf("IMU %d accel calibration beginning now!\n",IMU_ID);
      ICM_Data_Holder.ax_offset = 0;
      ICM_Data_Holder.ay_offset = 0;
      ICM_Data_Holder.az_offset = 0;//Zero all offsets to generate new set
      int sample_count = 0;
      while (sample_count < num_samples){
            if (interrupt){
                  //Interrupt triggered, count the loop
                  interrupt = false;//reset int
                  sample_count ++;
                  ICM_read_accel_gyro_lightweight(IMU_ID,true);
                  ICM_Data_Holder.ax_offset += ICM_Data_Holder.ax;
                  ICM_Data_Holder.ay_offset += ICM_Data_Holder.ay;
                  ICM_Data_Holder.az_offset += ICM_Data_Holder.az;
                  //The offset terms should accumilate
            }
      }
      //Now that the requsite samples are collected, average the offsets
      
      ICM_Data_Holder.ax_offset = ICM_Data_Holder.ax_offset/((float)sample_count);
      ICM_Data_Holder.ay_offset = ICM_Data_Holder.ay_offset/((float)sample_count);
      ICM_Data_Holder.az_offset = ICM_Data_Holder.az_offset/((float)sample_count);
      ICM_Data_Holder.az_offset = ICM_Data_Holder.az_offset - 9.81;//Subtract actual gravity 
      Serial.printf("IMU %d accel offsets: x: %.4f, y: %.4f, z: %.4f\n", IMU_ID, ICM_Data_Holder.ax_offset,ICM_Data_Holder.ay_offset,ICM_Data_Holder.az_offset);    

      Serial.println("Calibration complete! Input above values in initICM's case statement. The code will now stall. Remove line to calibrate accel to operate normally");

      while(1){
            delay(10000);
      }
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

