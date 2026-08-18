/**
 * \file        MPU6050_MF3000.cpp
 * \brief       Library for interfacing with MPU6050.
 *
 * \authors     Chase Sun
 * \bug
 */

// INCLUDES
#include "MegaStrain5000.h"

//GLOBAL VARIABLES
MegaStrain_Data MS5K_Data_Holder[NUM_MS5K];//The holder for IMU information. This declaration zeros all fields

bool IMU_first_read = true;

uint32_t lastMicros;//Used to time the kalman filter

int numIterations = 0; //Track how many iterations we have gone through


// FUNCTIONS

/**
 * \brief Tool to write I2C command to a register of a specific ID
 *
 * \param IMU_ID //ID of IMU
 * \param reg //The i2c register to write to
 * \param device_addr //The address of the device, ICM, MLX, or BMP
 * \param data //The data to write to
 * \return
 */
void MS5KWrite(int IMU_ID, uint8_t device_addr, uint8_t reg, uint8_t data){
        //First check if IMU_ID is valid
        if (IMU_ID < 0 || IMU_ID > ((int)NUM_MS5K-1)){
                return;//Just return fail read if the ID is not valid
        }
        MS5K_Data_Holder[IMU_ID].i2cBus->beginTransmission(device_addr);
        MS5K_Data_Holder[IMU_ID].i2cBus->write(reg);
        MS5K_Data_Holder[IMU_ID].i2cBus->write(data);
        MS5K_Data_Holder[IMU_ID].i2cBus->endTransmission(true); // We will free the wire every time we communicate. Its a bit slower
}

/**
 * \brief Tool to transceive MLX data. The write is always paired with a read and vice versa
 *
 * \param IMU_ID //ID of IMU
 * \param device_addr //The address of the MLX
 * \param txbuf
 * \param txlen
 * \param rxbuf
 * \param rxlen
 * \param interdelay
 * \return //Status byte
 */
uint8_t MLX_transceive(int IMU_ID, uint8_t device_addr, uint8_t *txbuf, uint8_t txlen, uint8_t *rxbuf, uint8_t rxlen = 0, uint8_t interdelay){

        uint8_t status = 0;
        uint8_t i;
        uint8_t rxbuf2[rxlen + 2];

        /* Write stage */
        MS5K_Data_Holder[IMU_ID].i2cBus->beginTransmission(device_addr);
        MS5K_Data_Holder[IMU_ID].i2cBus->write(txbuf, txlen);
        MS5K_Data_Holder[IMU_ID].i2cBus->endTransmission(false);
        delay(interdelay);
        /* Read status byte plus any others */
        MS5K_Data_Holder[IMU_ID].i2cBus->requestFrom(static_cast<uint8_t>(device_addr), static_cast<size_t>(rxlen + 1),true);
        for (int i = 0; i < rxlen + 1; ++i) rxbuf2[i] = MS5K_Data_Holder[IMU_ID].i2cBus->read();
        

        status = rxbuf2[0];
        for (i = 0; i < rxlen; i++) {
                rxbuf[i] = rxbuf2[i + 1];
        }

        return (status);
}

/**
 * \brief Tool to write to a register with MLX90393
 *
 * \param IMU_ID //ID of IMU
 * \param device_addr //The address of the MLX
 * \param reg // register address
 * \param data // data to be sent
 * \param hbyte //high byte
 * \param lbyte //low byte
 * \return //Status byte
 */
uint8_t MLX_writeRegister(int IMU_ID, uint8_t device_addr, uint8_t reg, uint8_t hbyte, uint8_t lbyte){
        uint8_t tx[4] = {
                CMD_WR_MLX,
                hbyte,   // high byte
                lbyte, // low byte
                (uint8_t)(reg << 2)};   // the register itself, shift up by 2 bits!

        return MLX_transceive(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, tx, sizeof(tx), NULL, 0, 0);
}

/**
 * \brief Tool to write to a register with MLX90393
 *
 * \param IMU_ID //ID of IMU
 * \param device_addr //The address of the MLX
 * \param reg // register address
 * \param data // data to be sent
 * \param hbyte //high byte
 * \param lbyte //low byte
 * \return //Status byte
 */
uint8_t MLX_readRegister(int IMU_ID, uint8_t device_addr, uint8_t reg, uint8_t *hbyte, uint8_t *lbyte){
        uint8_t tx[2] = {
                CMD_RR_MLX,
                (uint8_t)(reg << 2)}; // the register itself, shift up by 2 bits!

        uint8_t rx[2];

        /* Perform the transaction. */
        uint8_t status = MLX_transceive(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, tx, sizeof(tx), rx, sizeof(rx), 0);

        *hbyte = rx[0]; 
        *lbyte = rx[1];

        return status;
}

/**
 * \brief Tool to read I2C register. This only reads 1 register for checking config. Do not use this to read actual Accel/Gyro/Mag data as its too slow
 *
 * \param IMU_ID //ID of IMU
 * \param device_addr //Address of device, ICM / MLX / BMP
 * \param reg //The i2c register to write to
 * \return
 */
uint8_t MS5KRead(int IMU_ID, uint8_t device_addr, uint8_t reg) {
        //First check if IMU_ID is valid
        if (IMU_ID < 0 || IMU_ID > ((int)NUM_MS5K-1)){
                return 0;//Just return fail read if the ID is not valid
        }
        
        MS5K_Data_Holder[IMU_ID].i2cBus->beginTransmission(device_addr);
        MS5K_Data_Holder[IMU_ID].i2cBus->write(reg);
        MS5K_Data_Holder[IMU_ID].i2cBus->endTransmission(false);          // repeated START
        MS5K_Data_Holder[IMU_ID].i2cBus->requestFrom(device_addr,(size_t)1,true);
        return MS5K_Data_Holder[IMU_ID].i2cBus->read();
}


/**
 * \brief Function to initialize a single ICM42607
 *
 * \param IMU_ID the ID of the IMU. 0 = back, 1 = shank_L, 2 = thigh_L, 3 = shank_R, 4 = thigh_R
 * \param reportFreqnency the frequency to report data. If ran in IMU mode, this determines the frequency of the control loop
 * \return boolean value true meaning successfully initialized and false failed
 */
bool initICM(int IMU_ID, int reportFrequency) {
        
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
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG0_ICM,  0x45); // 1600
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG0_ICM, 0x25);
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG1_ICM,  0x01); //Bandwidth 180hz
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG1_ICM, 0x01);
        }
        else if(reportFrequency == 800){
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG0_ICM,  0x46); //800
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG0_ICM, 0x26);
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG1_ICM,  0x01); //Bandwidth 180hz
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG1_ICM, 0x01);
        }
        else if(reportFrequency == 400){
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG0_ICM,  0x47); // 400
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG0_ICM, 0x27);
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG1_ICM,  0x01); //Bandwidth 34hz
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG1_ICM, 0x05);
        }
        else if(reportFrequency == 200){//Microstrain equivalent settings for comparison
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG0_ICM,  0x28);//1000dps, 200hz, matching microstrain
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG0_ICM, 0x28);//8g, 200hz, matching microstrain
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG1_ICM,  0x02);//half of SR, 121hz, microstrain 100hz
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG1_ICM, 0x03);//half of SR, 73hz, microstrain 100hz
        }
        else if(reportFrequency == 100){
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG0_ICM,  0x49); //100
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG0_ICM, 0x29);
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, GYRO_CONFIG1_ICM,  0x04); //Bandwidth 16hz
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, ACCEL_CONFIG1_ICM, 0x04);
        }
        else{
                Serial.println("Report frequency can be only 1600 / 800 / 400 / 200 / 100 hz.");
                return false;
        }
        
        //Set up the interrupts
        //Set in pulse mode, push pull, active high, 0b00 011011 0x1B
        MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, INT_CONFIG_ICM, 0x1B);
        
        //Set int 2, 0b00001000, data ready
        //MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, INT_SOURCE3_ICM, 0x08);
        
        if (IMU_ID == 0){ //Back IMU
                //Set int 1 as well
                MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, INT_SOURCE0_ICM, 0x08);
        }

        //Set power mode
        // 0b0000 1111
        MS5KWrite(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, PWR_MGMT0_ICM, 0x0F);
        delay(100);//Mandatory pause after gyro turning on
        
        //Check if we are communicating with the right chip
        uint8_t check_addr = MS5KRead(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_IMU, WHO_AM_I_ICM);
        if (check_addr != 0x60){
                Serial.printf("Wrong address on ICM %d! Address found to be %x, address we are looking for is %x\n", IMU_ID, check_addr, MS5K_Data_Holder[IMU_ID].addr_IMU);
                return false; //Checking who am I failed, IMU not initialized
        }

        MS5K_Data_Holder[IMU_ID].valid_flag = 1;//Mark that the IMU is valid and in use

        //Last step: set the kalman filter parameters (defaults used)
        MS5K_Data_Holder[IMU_ID].kalmanRoll.setQangle(0.001);
        MS5K_Data_Holder[IMU_ID].kalmanRoll.setQbias(0.003);
        MS5K_Data_Holder[IMU_ID].kalmanRoll.setRmeasure(0.03);

        MS5K_Data_Holder[IMU_ID].kalmanPitch.setQangle(0.001);
        MS5K_Data_Holder[IMU_ID].kalmanPitch.setQbias(0.003);
        MS5K_Data_Holder[IMU_ID].kalmanPitch.setRmeasure(0.03);

        return true;
}

/**
 * \brief Function to initialize a single MLX90393
 *
 * \param IMU_ID the ID of the IMU. 0 = back, 1 = shank_L, 2 = thigh_L, 3 = shank_R, 4 = thigh_R
 * \return boolean value true meaning successfully initialized and false failed
 */
bool initMLX(int IMU_ID) {
        //For configuration, we do 3 steps
        //1. reset mlx and check for confit
        //2. configure registers 0, 1, and 2
        //3. Start the single read mode


        //Reset

        uint8_t tx_reset[1] = {CMD_RESET_MLX};

        uint8_t status = MLX_transceive(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, tx_reset, sizeof(tx_reset), NULL, 0, 5);
        Serial.printf("Status after reset: 0x%x\n",status);
        delay(10);


        //Configure registers
        //Reg 0 - gain
        uint8_t hbyte = 0x00;
        uint8_t lbyte = 0x00;

        status = MLX_readRegister(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, REG_0_ADDR_MLX, &hbyte, &lbyte);
        Serial.printf("Reg 0 before modification: Status: 0x%x, high byte: 0x%x, low byte: 0x%x\n", status, hbyte, lbyte);
        lbyte = 0x7C; // Set the high byte to 1x gain (0111)
        status = MLX_writeRegister(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, REG_0_ADDR_MLX, hbyte, lbyte);
        
        //Reg 1 - trigger mode
        hbyte = 0x00; //0000 1000 for external trigger or all zeros for manual i2c trigger
        lbyte = 0x00; //0000 0000 no burst mode
        status = MLX_writeRegister(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, REG_1_ADDR_MLX, hbyte, lbyte);
        
        //Reg 2 - resolution, filter, oversampling
        hbyte = 0x00; //All zeros for fastest acquisition
        lbyte = 0x01; 
        status = MLX_writeRegister(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, REG_2_ADDR_MLX, hbyte, lbyte);
        
        //Check the registers for settings
        status = MLX_readRegister(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, REG_0_ADDR_MLX, &hbyte, &lbyte);
        Serial.printf("Reg 0: Status: 0x%x, high byte: 0x%x, low byte: 0x%x\n", status, hbyte, lbyte);
        status = MLX_readRegister(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, REG_1_ADDR_MLX, &hbyte, &lbyte);
        Serial.printf("Reg 1: Status: 0x%x, high byte: 0x%x, low byte: 0x%x\n", status, hbyte, lbyte);
        status = MLX_readRegister(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, REG_2_ADDR_MLX, &hbyte, &lbyte);
        Serial.printf("Reg 2: Status: 0x%x, high byte: 0x%x, low byte: 0x%x\n", status, hbyte, lbyte);        

        //Start the single write command
        uint8_t tx_single[1] = {0x3E};
        //status = MLX_transceive(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, tx_single, sizeof(tx_single), NULL, 0, 0);
        //Serial.printf("Status after setting to single mode: 0x%x\n",status);
        delay(15);

        //status = MLX_read_mag_lightweight(IMU_ID);

        delay(10);

        MS5K_Data_Holder[IMU_ID].kalmanYaw.setQangle(0.001);
        MS5K_Data_Holder[IMU_ID].kalmanYaw.setQbias(0.003);
        MS5K_Data_Holder[IMU_ID].kalmanYaw.setRmeasure(0.03);

        return true;
}

/**
 * \brief Function to initialize all IMUs. If an IMU is not active, it will be marked down and not used.
 * \param reportFrequency The report rate of imus which controls clock rate of the low level controller
 * \param 
 * \return nothing
 */
 void MegaStrain_bulk_initialization(int reportFrequency){
        
        for (int i = 0; i < (int)NUM_MS5K; i++){//The forloop initializes all 5 IMUs. It is fine if IMUs are not used, the code will just mark them invalid and skip them in reading data
                //First we initialze the struct and set ID and wire
                MS5K_Data_Holder[i].valid_flag = 0;//assume imu is not used
                switch(i){
                        case 0://Back
                                MS5K_Data_Holder[i].addr_IMU = 0x69;
                                MS5K_Data_Holder[i].addr_MAG = 0x0F;
                                MS5K_Data_Holder[i].addr_BMP = 0x47;
                                MS5K_Data_Holder[i].i2cBus = &Wire2;
                                MS5K_Data_Holder[i].ax_offset = 0;//These are factory offsets generated by the accel factory calibration function. Update when IMU is swapped
                                MS5K_Data_Holder[i].ay_offset = 0;
                                MS5K_Data_Holder[i].az_offset = 0;
                                break;
                        case 1://L_shank
                                MS5K_Data_Holder[i].addr_IMU = 0x68;
                                MS5K_Data_Holder[i].addr_MAG = 0x0E;
                                MS5K_Data_Holder[i].addr_BMP = 0x46;
                                MS5K_Data_Holder[i].i2cBus = &Wire;
                                MS5K_Data_Holder[i].ax_offset = 0;
                                MS5K_Data_Holder[i].ay_offset = 0;
                                MS5K_Data_Holder[i].az_offset = 0;
                                break;
                        case 2://L_thigh
                                MS5K_Data_Holder[i].addr_IMU = 0x69;
                                MS5K_Data_Holder[i].addr_MAG = 0x0F;
                                MS5K_Data_Holder[i].addr_BMP = 0x47;
                                MS5K_Data_Holder[i].i2cBus = &Wire;
                                MS5K_Data_Holder[i].ax_offset = 0;
                                MS5K_Data_Holder[i].ay_offset = 0;
                                MS5K_Data_Holder[i].az_offset = 0;
                                break;
                        case 3://R_shank
                                MS5K_Data_Holder[i].addr_IMU = 0x68;
                                MS5K_Data_Holder[i].addr_MAG = 0x0E;
                                MS5K_Data_Holder[i].addr_BMP = 0x46;
                                MS5K_Data_Holder[i].i2cBus = &Wire1;
                                MS5K_Data_Holder[i].ax_offset = 0;
                                MS5K_Data_Holder[i].ay_offset = 0;
                                MS5K_Data_Holder[i].az_offset = 0;
                                break;
                        case 4://R_thigh
                                MS5K_Data_Holder[i].addr_IMU = 0x69;
                                MS5K_Data_Holder[i].addr_MAG = 0x0F;
                                MS5K_Data_Holder[i].addr_BMP = 0x47;
                                MS5K_Data_Holder[i].i2cBus = &Wire1;
                                MS5K_Data_Holder[i].ax_offset = 0;
                                MS5K_Data_Holder[i].ay_offset = 0;
                                MS5K_Data_Holder[i].az_offset = 0;
                                break;
                        default://Invalid
                                return;
                }

                if(initICM(i,reportFrequency) == 0){Serial.printf("Failed to initialize ICM %d. Marked disabled, skipping\n", i);}
                else{Serial.printf("ICM %d initialized with frequency %d\n", i, reportFrequency);}

                if(initMLX(i) == 0){Serial.printf("Failed to initialize MLX %d. Skipping\n", i);}
                else{Serial.printf("MLX %d initialized\n", i);}

                

                
        }
 }


/**
 * \brief Function to calibrate the gyroscope of all sensors simutaniously. The function populates the gx_offset, gy_offset, and gz_offset data fields in the imu struct
 * takes advantage of the interrupt and ISR to time reads. Calibrates all online sensors at once
 * \param num_samples the number of samples to average. Default 400
 * \param interrupt the ISR boolean used as a clock
 * \return nothing
 */
 void ICM_bulk_calibration(int num_samples, volatile bool& interrupt){
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
 * \brief Function to read IMU data with low level i2c code. The reason why adafruit's getEvent is not used is because it is too slow due to it reading multiple i2c registers to find out imu settings for calcuation
 *When the function is completed, the imu data struct will be populated with the most recent data
 * \param IMU_ID the id of the imu
 * \param calibrationMode boolean to select if gyroscope is in calibration. Default is false and only choose true if in initial calibration step.
 * \return boolean whether read was successful
 */
 bool ICM_read_accel_gyro_lightweight(int IMU_ID, bool calibrationMode){
        //First check if IMU_ID is valid
        if (IMU_ID < 0 || IMU_ID > ((int)NUM_MS5K-1)){
                return 0;//Just return fail read if the ID is not valid
        }
        //First check if IMU is actually initialized
        if (MS5K_Data_Holder[IMU_ID].valid_flag == false){
                return 0;//Just return fail read if the IMU is not being used
        }

        // Burst read 12 bytes starting at ACCEL_DATA_X1 (0x0B)
        uint8_t buf[12];

        MS5K_Data_Holder[IMU_ID].i2cBus->beginTransmission(MS5K_Data_Holder[IMU_ID].addr_IMU);
       
        MS5K_Data_Holder[IMU_ID].i2cBus->write(DATA_START_ICM);

        MS5K_Data_Holder[IMU_ID].i2cBus->endTransmission(false);
        //if (MS5K_Data_Holder[IMU_ID].i2cBus->requestFrom(MS5K_Data_Holder[IMU_ID].addr_ICM, (uint8_t)12) != 12) return false;
        
        MS5K_Data_Holder[IMU_ID].i2cBus->requestFrom(static_cast<uint8_t>(MS5K_Data_Holder[IMU_ID].addr_IMU),static_cast<size_t>(12),true);
        //MS5K_Data_Holder[IMU_ID].i2cBus->requestFrom(MS5K_Data_Holder[IMU_ID].addr_ICM, 12, true);

        for (int i = 0; i < 12; ++i) buf[i] = MS5K_Data_Holder[IMU_ID].i2cBus->read();

        // Unpack raw 16-bit values
        int16_t ax_raw = (int16_t)((buf[0]  << 8) | buf[1]);
        int16_t ay_raw = (int16_t)((buf[2]  << 8) | buf[3]);
        int16_t az_raw = (int16_t)((buf[4]  << 8) | buf[5]);
        
        int16_t gx_raw = (int16_t)((buf[6]  << 8) | buf[7]);
        int16_t gy_raw = (int16_t)((buf[8]  << 8) | buf[9]);
        int16_t gz_raw = (int16_t)((buf[10] << 8) | buf[11]);

        // --- Conversion constants (match your configured ranges) ---
        // Accel LSB per g: ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048
        const float ACCEL_LSB_PER_G = 4096.0f;     // for ±8g
        const float GYRO_LSB_PER_DPS = 32.8f;      // for ±1000 dps
        const float G = 9.80665f;

        
        //Make distinction in calibration mode
        if (calibrationMode){
                //If we are calibrating - should not be the case during normal operation
                //We will accumilate the measurements in the calibration field to be averaged later
                MS5K_Data_Holder[IMU_ID].gx = gx_raw / GYRO_LSB_PER_DPS;
                MS5K_Data_Holder[IMU_ID].gy = gy_raw / GYRO_LSB_PER_DPS;
                MS5K_Data_Holder[IMU_ID].gz = gz_raw / GYRO_LSB_PER_DPS;

                MS5K_Data_Holder[IMU_ID].ax = (ax_raw / ACCEL_LSB_PER_G) * G;
                MS5K_Data_Holder[IMU_ID].ay = (ay_raw / ACCEL_LSB_PER_G) * G;
                MS5K_Data_Holder[IMU_ID].az = (az_raw / ACCEL_LSB_PER_G) * G;

                MS5K_Data_Holder[IMU_ID].gx_offset += MS5K_Data_Holder[IMU_ID].gx;
                MS5K_Data_Holder[IMU_ID].gy_offset += MS5K_Data_Holder[IMU_ID].gy;
                MS5K_Data_Holder[IMU_ID].gz_offset += MS5K_Data_Holder[IMU_ID].gz;
        }
        else{
                //Not in calibration mode, we either have valid constants or they are zero, subtract regardless
                MS5K_Data_Holder[IMU_ID].gx = gx_raw / GYRO_LSB_PER_DPS - MS5K_Data_Holder[IMU_ID].gx_offset;
                MS5K_Data_Holder[IMU_ID].gy = gy_raw / GYRO_LSB_PER_DPS - MS5K_Data_Holder[IMU_ID].gy_offset;
                MS5K_Data_Holder[IMU_ID].gz = gz_raw / GYRO_LSB_PER_DPS - MS5K_Data_Holder[IMU_ID].gz_offset;

                MS5K_Data_Holder[IMU_ID].ax = (ax_raw / ACCEL_LSB_PER_G) * G - MS5K_Data_Holder[IMU_ID].ax_offset;//Apply hardcoded factory offset
                MS5K_Data_Holder[IMU_ID].ay = (ay_raw / ACCEL_LSB_PER_G) * G - MS5K_Data_Holder[IMU_ID].ay_offset;
                MS5K_Data_Holder[IMU_ID].az = (az_raw / ACCEL_LSB_PER_G) * G - MS5K_Data_Holder[IMU_ID].az_offset;
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
 */
 void ICM_factory_accel_calibration(int IMU_ID, int num_samples, volatile bool& interrupt){
        //First check if IMU_ID is valid
        if (IMU_ID < 0 || IMU_ID > ((int)NUM_MS5K-1)){
                return;//Just return fail read if the ID is not valid
        }
        //First check if IMU is actually initialized
        if (MS5K_Data_Holder[IMU_ID].valid_flag == false){
                return;//Just return fail read if the IMU is not being used
        }
        
        Serial.printf("IMU %d accel calibration beginning in 3 seconds. Keep it flat and face up\n",IMU_ID);
        delay(1000);
        Serial.printf("IMU %d accel calibration beginning in 2 seconds. Keep it flat and face up\n",IMU_ID);
        delay(1000);
        Serial.printf("IMU %d accel calibration beginning in 1 seconds. Keep it flat and face up\n",IMU_ID);
        delay(1000);
        Serial.printf("IMU %d accel calibration beginning now!\n",IMU_ID);
        MS5K_Data_Holder[IMU_ID].ax_offset = 0;
        MS5K_Data_Holder[IMU_ID].ay_offset = 0;
        MS5K_Data_Holder[IMU_ID].az_offset = 0;//Zero all offsets to generate new set
        int sample_count = 0;
        while (sample_count < num_samples){
                if (interrupt){
                        //Interrupt triggered, count the loop
                        interrupt = false;//reset int
                        sample_count ++;
                        ICM_read_accel_gyro_lightweight(IMU_ID,true);
                        MS5K_Data_Holder[IMU_ID].ax_offset += MS5K_Data_Holder[IMU_ID].ax;
                        MS5K_Data_Holder[IMU_ID].ay_offset += MS5K_Data_Holder[IMU_ID].ay;
                        MS5K_Data_Holder[IMU_ID].az_offset += MS5K_Data_Holder[IMU_ID].az;
                        //The offset terms should accumilate
                }
        }
        //Now that the requsite samples are collected, average the offsets
        
        MS5K_Data_Holder[IMU_ID].ax_offset = MS5K_Data_Holder[IMU_ID].ax_offset/((float)sample_count);
        MS5K_Data_Holder[IMU_ID].ay_offset = MS5K_Data_Holder[IMU_ID].ay_offset/((float)sample_count);
        MS5K_Data_Holder[IMU_ID].az_offset = MS5K_Data_Holder[IMU_ID].az_offset/((float)sample_count);
        MS5K_Data_Holder[IMU_ID].az_offset = MS5K_Data_Holder[IMU_ID].az_offset - 9.81;//Subtract actual gravity 
        Serial.printf("IMU %d accel offsets: x: %.4f, y: %.4f, z: %.4f\n", IMU_ID, MS5K_Data_Holder[IMU_ID].ax_offset,MS5K_Data_Holder[IMU_ID].ay_offset,MS5K_Data_Holder[IMU_ID].az_offset);    

        Serial.println("Calibration complete! Input above values in initICM's case statement. The code will now stall. Remove line to calibrate accel to operate normally");

        while(1){
                delay(10000);
        }
 }

 /**
 * \brief Function to trigger all mlx reads
 * \return nothing
 */
void MLX_bulk_trigger_mag_lightweight(){
        //This function writes the single measurement command to all IMUs
        //It should be called at the start of a loop
        //Serial.println("Bulk triggering mlx:");
        uint8_t tx_single[1] = {CMD_SM_XYZ_MLX};

         for (int i = 0; i < (int)NUM_MS5K; i++){
                if (MS5K_Data_Holder[i].valid_flag == true){
        
                        uint8_t status = MLX_transceive(i, MS5K_Data_Holder[i].addr_MAG, tx_single, sizeof(tx_single), NULL, 0, 0);
                        //Serial.printf("Status after triggering mag %d to single mode: 0x%x\n",i,status);
                }
        }
 }

 /**
 * \brief Function to read a single mlx xyz mag reading
 * \param IMU_ID id of the IMU to be read
 * \return //The status byte
 */
 uint8_t MLX_read_mag_lightweight(int IMU_ID, bool calibrationMode){
       //First check if IMU_ID is valid
        if (IMU_ID < 0 || IMU_ID > ((int)NUM_MS5K-1)){
                return 0;//Just return fail read if the ID is not valid
        }
        //First check if IMU is actually initialized
        if (MS5K_Data_Holder[IMU_ID].valid_flag == false){
                return 0;//Just return fail read if the IMU is not being used
        }

        uint8_t tx_read[1] = {CMD_RM_XYZ_MLX};
        uint8_t tx_check[1] = {0x00};//Blank command to check
        uint8_t rx[6] = {0};

        // Burst read 7 bytes - 1 byte status, 6 bytes of 16 bit xyz
        
        uint8_t status = 0;
        uint8_t error_bit = 1;
        
        //while(1){
                
                //status = MLX_transceive(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, tx_check, sizeof(tx_check), NULL, 0, 0);
                status = MLX_transceive(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, tx_read, sizeof(tx_read), rx, sizeof(rx), 0);
                error_bit = status & 0x10;
                if (error_bit == 0){
                        //break;
                }
                //Serial.printf("Status: 0x%x\n",status);
        //}
        //Serial.printf("Status after success: 0x%x\n",status);

        //status = MLX_transceive(IMU_ID, MS5K_Data_Holder[IMU_ID].addr_MAG, tx_read, sizeof(tx_read), rx, sizeof(rx), 0);

        // Unpack raw 16-bit values
        int16_t mx_raw = (int16_t)((rx[0]  << 8) | rx[1]);
        int16_t my_raw = (int16_t)((rx[2]  << 8) | rx[3]);
        int16_t mz_raw = (int16_t)((rx[4]  << 8) | rx[5]);

        

        float mx_unfilt = (float)mx_raw * 0.150;//from mlx documentation
        float my_unfilt = (float)my_raw * 0.150;
        float mz_unfilt = (float)mz_raw * 0.242;


        if (IMU_first_read){
                for (size_t i = 0; i < Moving_average_windowSize; ++i) {
                        MS5K_Data_Holder[IMU_ID].mx_array[i] = 0;
                        MS5K_Data_Holder[IMU_ID].my_array[i] = 0;
                        MS5K_Data_Holder[IMU_ID].mz_array[i] = 0;
                        MS5K_Data_Holder[IMU_ID].ma_index_x = 0;
                        MS5K_Data_Holder[IMU_ID].ma_index_y = 0;
                        MS5K_Data_Holder[IMU_ID].ma_index_z = 0;

                        MS5K_Data_Holder[IMU_ID].mx_max = -99999; //For calibration purposes, ignore if in normal operation
                        MS5K_Data_Holder[IMU_ID].mx_min = 99999;
                        MS5K_Data_Holder[IMU_ID].my_max = -99999;
                        MS5K_Data_Holder[IMU_ID].my_min = 99999;
                        MS5K_Data_Holder[IMU_ID].mz_max = -99999;
                        MS5K_Data_Holder[IMU_ID].mz_min = 99999;
                }
        }

        MS5K_Data_Holder[IMU_ID].mx = moving_average(MS5K_Data_Holder[IMU_ID].mx_array, mx_unfilt, MS5K_Data_Holder[IMU_ID].ma_index_x);//from mlx documentation
        MS5K_Data_Holder[IMU_ID].my = moving_average(MS5K_Data_Holder[IMU_ID].my_array, my_unfilt, MS5K_Data_Holder[IMU_ID].ma_index_y);
        MS5K_Data_Holder[IMU_ID].mz = moving_average(MS5K_Data_Holder[IMU_ID].mz_array, mz_unfilt, MS5K_Data_Holder[IMU_ID].ma_index_z);

        if (!calibrationMode){ //not in calibration, normal operation, apply hard iron and soft iron values
                MS5K_Data_Holder[IMU_ID].mx = (MS5K_Data_Holder[IMU_ID].mx - MLX_Hardiron[IMU_ID][0])*MLX_Softiron[IMU_ID][0];
                MS5K_Data_Holder[IMU_ID].my = (MS5K_Data_Holder[IMU_ID].my - MLX_Hardiron[IMU_ID][1])*MLX_Softiron[IMU_ID][1];
                MS5K_Data_Holder[IMU_ID].mz = (MS5K_Data_Holder[IMU_ID].mz - MLX_Hardiron[IMU_ID][2])*MLX_Softiron[IMU_ID][2];
        }

        //Below is calibration code for soft/hard iron. Do not use in normal operation
        numIterations ++;
        
        if (calibrationMode && numIterations > 4000){//Only trigger calibration after 10 seconds at 400 hz
                if (MS5K_Data_Holder[IMU_ID].mx > MS5K_Data_Holder[IMU_ID].mx_max){
                        MS5K_Data_Holder[IMU_ID].mx_max = MS5K_Data_Holder[IMU_ID].mx;
                }
                if (MS5K_Data_Holder[IMU_ID].mx < MS5K_Data_Holder[IMU_ID].mx_min){
                        MS5K_Data_Holder[IMU_ID].mx_min = MS5K_Data_Holder[IMU_ID].mx;
                }

                if (MS5K_Data_Holder[IMU_ID].my > MS5K_Data_Holder[IMU_ID].my_max){
                        MS5K_Data_Holder[IMU_ID].my_max = MS5K_Data_Holder[IMU_ID].my;
                }
                if (MS5K_Data_Holder[IMU_ID].my < MS5K_Data_Holder[IMU_ID].my_min){
                        MS5K_Data_Holder[IMU_ID].my_min = MS5K_Data_Holder[IMU_ID].my;
                }

                if (MS5K_Data_Holder[IMU_ID].mz > MS5K_Data_Holder[IMU_ID].mz_max){
                        MS5K_Data_Holder[IMU_ID].mz_max = MS5K_Data_Holder[IMU_ID].mz;
                }
                if (MS5K_Data_Holder[IMU_ID].mz < MS5K_Data_Holder[IMU_ID].mz_min){
                        MS5K_Data_Holder[IMU_ID].mz_min = MS5K_Data_Holder[IMU_ID].mz;
                }

        }

        return status;
}


/**
 * \brief Function to read all IMUs at once. If an IMU is not active, the function simply does not populate the data field of the struct.
 *This function should be called when ISR is triggered. This is a non-blocking read in the sense that it will read whatever data is available in the IMU data registers no matter new or old. Therefore, it can be called at any frequency independent of the IMU sample rate. However, the read speed limited by i2c speed are listed below
 *| I2C HS       |400,000|       |0.4 ms per read|       |2 ms total|
 *| I2C HS+      |1,000,000|     |0.2 ms per read|       |1 ms total|
 *| I2C LS       |100,000|       |2.0 ms per read|       |10 ms total|
 * \param calibrationMode boolean to select if gyroscope is in calibration. Default is false and only choose true if in initial calibration step.

 * \return nothing
 */
 void read_all_MS5K(bool calibrationMode){

        for (int i = 0; i < (int)NUM_MS5K; i++){
                
                MLX_read_mag_lightweight(i,calibrationMode);
                
        }

        MLX_bulk_trigger_mag_lightweight();

        for (int i = 0; i < (int)NUM_MS5K; i++){
                ICM_read_accel_gyro_lightweight(i,calibrationMode);
        }
        //We read ICM first then MLX to give the MLX some time to perform the conversion
        
        
}

/**
 * \brief Function to perform Kalman filtering of a single IMU. Populates the Euler angle fields of the IMU struct
 * the function will only perform kalman filtering if IMU is active
 * \param IMU_ID the id of the imu
 * \param dt the time elapsed from the last read for gyro integration
 * \param mode mode selector for flat (microstrain default) or upright (better for yaw sensing)
 * \return nothing
 */
 void Kalman_single_MS5K(int IMU_ID, float dt, int mode){
        //default mode 0 - flat mode, xyz axis stay in tact
        
        float ax = MS5K_Data_Holder[IMU_ID].ax;
        float ay = MS5K_Data_Holder[IMU_ID].ay;
        float az = MS5K_Data_Holder[IMU_ID].az;

        float gx = MS5K_Data_Holder[IMU_ID].gx;
        float gy = MS5K_Data_Holder[IMU_ID].gy;
        float gz = MS5K_Data_Holder[IMU_ID].gz;

        float mx = MS5K_Data_Holder[IMU_ID].mx;
        float my = MS5K_Data_Holder[IMU_ID].my;
        float mz = MS5K_Data_Holder[IMU_ID].mz;

        /*

        float ax = MS5K_Data_Holder[IMU_ID].ay;
        float ay = -MS5K_Data_Holder[IMU_ID].ax;
        float az = MS5K_Data_Holder[IMU_ID].az;

        float gx = MS5K_Data_Holder[IMU_ID].gy;
        float gy = -MS5K_Data_Holder[IMU_ID].gx;
        float gz = MS5K_Data_Holder[IMU_ID].gz;

        float mx = MS5K_Data_Holder[IMU_ID].my;
        float my = -MS5K_Data_Holder[IMU_ID].mx;
        float mz = MS5K_Data_Holder[IMU_ID].mz;

        */

        if (mode == 1){//Upright mode, y stays the same, raw z maps to x, x maps to negative z
                ax = MS5K_Data_Holder[IMU_ID].az;
                az = -1.0*MS5K_Data_Holder[IMU_ID].ax;

                gx = MS5K_Data_Holder[IMU_ID].gz;
                gz = -1.0*MS5K_Data_Holder[IMU_ID].gx;

                mx = MS5K_Data_Holder[IMU_ID].mz;
                mz = -1.0*MS5K_Data_Holder[IMU_ID].mx;
        }

        float rollAcc  = atan2(ay, az) * RAD_TO_DEG;
        float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

        
        if(IMU_first_read){
                MS5K_Data_Holder[IMU_ID].kalmanRoll.setAngle(rollAcc);
                MS5K_Data_Holder[IMU_ID].kalmanPitch.setAngle(pitchAcc);
                MS5K_Data_Holder[IMU_ID].roll = rollAcc;
                MS5K_Data_Holder[IMU_ID].pitch = pitchAcc; //If first reading, the roll and pitch will be purely based on accelerometer.
                //The first read boolean can also be held true to disable the kalman filter for debugging

                //Compute yaw using accel angle readings
                float magX = mx * cos(pitchAcc * DEG_TO_RAD) + mz * sin(pitchAcc * DEG_TO_RAD);
                float magY = mx * sin(rollAcc * DEG_TO_RAD)*sin(pitchAcc * DEG_TO_RAD) + my * cos(rollAcc * DEG_TO_RAD) - mz * sin(rollAcc * DEG_TO_RAD)*cos(pitchAcc * DEG_TO_RAD);

                float yawMag = atan2(-magY, magX) * RAD_TO_DEG;

                MS5K_Data_Holder[IMU_ID].kalmanPitch.setAngle(yawMag);

                MS5K_Data_Holder[IMU_ID].yaw = yawMag;
                
                return;
        }
        //Now that first read is over, we actually engage the kalman filter
        
        //Compute sensor fusion of roll and pitch first as we need them for 
        MS5K_Data_Holder[IMU_ID].roll = MS5K_Data_Holder[IMU_ID].kalmanRoll.getAngle(rollAcc, gx, dt);

        MS5K_Data_Holder[IMU_ID].pitch = MS5K_Data_Holder[IMU_ID].kalmanPitch.getAngle(pitchAcc, gy, dt);

        //We compute mag now with the kalman corrected pitch and roll
        float magX = mx * cos(MS5K_Data_Holder[IMU_ID].pitch * DEG_TO_RAD) + mz * sin(MS5K_Data_Holder[IMU_ID].pitch * DEG_TO_RAD);
        float magY = mx * sin(MS5K_Data_Holder[IMU_ID].roll * DEG_TO_RAD)*sin(MS5K_Data_Holder[IMU_ID].pitch * DEG_TO_RAD) + my * cos(MS5K_Data_Holder[IMU_ID].roll * DEG_TO_RAD) - mz * sin(MS5K_Data_Holder[IMU_ID].roll * DEG_TO_RAD)*cos(MS5K_Data_Holder[IMU_ID].pitch * DEG_TO_RAD);

        float yawMag = atan2(-magY, magX) * RAD_TO_DEG;

        if (yawMag - MS5K_Data_Holder[IMU_ID].yaw > 180) yawMag -= 360;
        else if (yawMag - MS5K_Data_Holder[IMU_ID].yaw < -180) yawMag += 360;
        
        MS5K_Data_Holder[IMU_ID].yaw = MS5K_Data_Holder[IMU_ID].kalmanYaw.getAngle(yawMag, gz, dt);


 }        
        

/**
 * \brief Function to kalman filter all available IMUs
 * \param mode mode selector for flat (microstrain) or upright mode
 * \return nothing
 */
 void Kalman_batch_MS5K(int mode){
        uint32_t currentMicros = micros();
        float dt = (currentMicros - lastMicros) * 1e-6;
        //Serial.printf("dt %.8f\n",dt);
        lastMicros = currentMicros;

        for (int i = 0; i < (int)NUM_MS5K; i++){
                Kalman_single_MS5K(i,dt,mode);
        }
        IMU_first_read = false;//Only triggers once after the first read. Remove this line to disable kalman filter and only use accel for roll/pitch
 }

/**
 * \brief Function to compute a moving average on the magnetometer
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

