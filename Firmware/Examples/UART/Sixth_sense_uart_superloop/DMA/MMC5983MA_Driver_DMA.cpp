/**
 * \file MMC5983MA_Driver.cpp
 * \brief OOP Library for interfacing with MMC5983MA.
 */

#include "MMC5983MA_Driver.h"

// Define the static pointer to allow static ISR functions to access class member data
MMC5983MA* MMC5983MA::instance = nullptr;

// Constructor
MMC5983MA::MMC5983MA(bool use_rtos, TaskHandle_t sensor_task) 
            : spi_dma{}, 
              data_holder{}, 
              dma_in_progress(false), 
              new_data_ready(false), 
              using_RTOS(use_rtos), 
              SensorTaskHandle(sensor_task) {
    
      instance = this;
}

// ISRs
void IRAM_ATTR MMC5983MA::ISR_dataReady() {
      if (instance && !instance->dma_in_progress) {
            if (spi_device_queue_trans(instance->spi_dma.handle, &instance->spi_dma.trans, 0) == ESP_OK) {
                  instance->dma_in_progress = true;
            }
      }
}

void IRAM_ATTR MMC5983MA::ISR_DMAcomplete_callback(spi_transaction_t *trans) {
      if (!instance) return;
      instance->new_data_ready = true;
      if (instance->using_RTOS) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (instance->SensorTaskHandle != NULL) {
                  vTaskNotifyGiveFromISR(instance->SensorTaskHandle, &xHigherPriorityTaskWoken);
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
      }
}

// SPI Register Access
void MMC5983MA::write_reg(uint8_t reg, uint8_t data) {
      spi_transaction_t t = {};
      t.flags = SPI_TRANS_USE_TXDATA;
      t.length = 16; 
      t.tx_data[0] = (0x00 | (reg & MMC_SPI_ADDR_MASK));
      t.tx_data[1] = data;
      spi_device_polling_transmit(spi_dma.handle, &t);
}

uint8_t MMC5983MA::read_reg(uint8_t reg) {
      spi_transaction_t t = {};
      t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      t.length = 16;
      t.tx_data[0] = (MMC_SPI_READ_FLAG | (reg & MMC_SPI_ADDR_MASK));
      t.tx_data[1] = 0x00; 
      spi_device_polling_transmit(spi_dma.handle, &t);
      return t.rx_data[1];
}

/**
 * \brief Validates configuration struct fields per datasheet constraints.
 */
bool MMC5983MA::check_config_validity(const MMC_Config_t &config) {
      bool is_valid = true;

      // Check Output Rate: 1 (1Hz) to 7 (1000Hz)
      if (config.outputRate < 1 || config.outputRate > 7) {
            Serial.printf("[MMC-CONFIG-ERROR] Invalid outputRate: %u (Expected 1 - 7)\n", config.outputRate);
            is_valid = false;
      }

      // Check Bandwidth: 1 (100Hz / 8ms) to 4 (800Hz / 0.5ms)
      if (config.bandwidth < 1 || config.bandwidth > 4) {
            Serial.printf("[MMC-CONFIG-ERROR] Invalid bandwidth: %u (Expected 1 - 4)\n", config.bandwidth);
            is_valid = false;
      }

      // Check Set Frequency: 0 (disabled) to 8 (2000 samples)
      if (config.setFrequency > 8) {
            Serial.printf("[MMC-CONFIG-ERROR] Invalid setFrequency: %u (Expected 0 - 8)\n", config.setFrequency);
            is_valid = false;
      }

      // Hardware constraints:
      // Rate 7 (1000 Hz) requires measurement time <= 0.5 ms -> BW setting 4 (800 Hz)
      if (config.outputRate == 7 && config.bandwidth != 4) {
            Serial.printf("[MMC-CONFIG-ERROR] 1000Hz (rate 7) requires bandwidth 4 (800Hz / 0.5ms)\n");
            is_valid = false;
      }
      // Rate 6 (200 Hz) requires measurement time <= 4 ms -> BW setting >= 2 (200 Hz / 4ms)
      if (config.outputRate == 6 && config.bandwidth < 2) {
            Serial.printf("[MMC-CONFIG-ERROR] 200Hz (rate 6) requires bandwidth >= 2 (200Hz / 4ms)\n");
            is_valid = false;
      }

      return is_valid;
}

/**
 * \brief Reads the device registers and populates the 1-indexed config struct.
 */
bool MMC5983MA::read_config(MMC_Config_t &out_config) {
      uint8_t ctrl1 = read_reg(MMC_CTRL1);
      uint8_t ctrl2 = read_reg(MMC_CTRL2);
      
      out_config.bandwidth  = (ctrl1 & 0x03) + 1;
      out_config.outputRate = (ctrl2 & 0x07);

      // Bit 7: En_prd_set. If 0, periodic set is disabled.
      if (ctrl2 & 0x80) {
            out_config.setFrequency = ((ctrl2 >> 4) & 0x07) + 1;
      } else {
            out_config.setFrequency = 0;
      }

      parse_config(out_config);

      return true;
}

/**
 * \brief Reads the device registers to populate the configuration struct.
 */
static void MMC5983MA::parse_config(const MMC_Config_t &config) {

      Serial.println("--- MMC5983MA Current Configuration ---");
      
      if (config.outputRate >= 1 && config.outputRate <= 7) {
            Serial.printf("Output Data Rate: %d Hz\n", freq_table[config.outputRate - 1]);
      } else {
            Serial.printf("Output Data Rate: Invalid (%u)\n", config.outputRate);
      }

      if (config.bandwidth >= 1 && config.bandwidth <= 4) {
            Serial.printf("Filter Bandwidth: %d Hz (%.1f ms duration)\n", 
                        bw_hz_table[config.bandwidth - 1], bw_time_table[config.bandwidth - 1]);
      } else {
            Serial.printf("Filter Bandwidth: Invalid (%u)\n", config.bandwidth);
      }

      if (config.setFrequency == 0) {
            Serial.println("Periodic SET Interval: Disabled");
      } else if (config.setFrequency >= 1 && config.setFrequency <= 8) {
            Serial.printf("Periodic SET Interval: Every %d measurements\n", set_samples_table[config.setFrequency - 1]);
      } else {
            Serial.printf("Periodic SET Interval: Invalid (%u)\n", config.setFrequency);
      }

      Serial.println("---------------------------------------");
}

/**
 * \brief Initializes the MMC5983MA into continuous measurement mode with explicit config.
 */
bool MMC5983MA::init_chip(const MMC_Config_t &config) {

      if (!config.chip_enable){
            Serial.println("[MMC] MMC chip disabled");
            return false;
      }

      if (!check_config_validity(config)) {
            Serial.println("[MMC-ERROR] Invalid configuration parameters. Aborting Init.");
            return false;
      }

      // Initialize offsets
      data_holder.offset[0] = 0.0f;
      data_holder.offset[1] = 0.0f;
      data_holder.offset[2] = 0.0f;

      // Initialize identity matrix for soft iron
      data_holder.W[0][0] = 1.0f; data_holder.W[0][1] = 0.0f; data_holder.W[0][2] = 0.0f;
      data_holder.W[1][0] = 0.0f; data_holder.W[1][1] = 1.0f; data_holder.W[1][2] = 0.0f;
      data_holder.W[2][0] = 0.0f; data_holder.W[2][1] = 0.0f; data_holder.W[2][2] = 1.0f;

      //Initialize last measurement
      mag_last_measurement[0] = 0;
      mag_last_measurement[0] = 0;
      mag_last_measurement[0] = 0;

      data_holder.newData = false;//Assume no new data will come in unless proven otherwise

      // Initialize SPI bus
      if (!SPI_Bus_Init(PIN_MOSI_MMC, PIN_MISO_MMC, PIN_SCLK_MMC)) {
            Serial.println("[MMC-ERROR] SPI host initialization failed!");
            return false;
      }
      if (!SPI_Add_Device(PIN_CS_MMC, ISR_DMAcomplete_callback, spi_dma.handle)) {
            Serial.println("[MMC-ERROR] Could not add MMC Device!");
            return false;
      }
      if (!SPI_Arm_DMA_Channel(MMC_BURST_LEN, ((MMC_XOUT0 & MMC_SPI_ADDR_MASK) | MMC_SPI_READ_FLAG), spi_dma)) {
            Serial.println("[MMC-ERROR] DMA setup for MMC failed!");
            return false;
      }

      // 1. Issue Software Reset
      write_reg(MMC_CTRL1, 0x80); 
      delay(15); 

      // 2. Verify Product ID
      uint8_t prod_id = read_reg(MMC_PROD_ID);
      if (prod_id != 0x30) {
            Serial.printf("[MMC-ERROR] Invalid Product ID: 0x%02X (Expected 0x30)\n", prod_id);
            return false;
      }

      // 3. Perform an initial SET pulse
      write_reg(MMC_CTRL0, 0x08); 
      delay(1); 

      // 4. Update frequency state tracking
      ODR = freq_table[config.outputRate - 1];

      // 5. Apply the Bandwidth setting to Internal Control 1 (0x0A): convert 1-4 -> 0-3
      uint8_t bw_reg = (config.bandwidth - 1) & 0x03;
      write_reg(MMC_CTRL1, bw_reg);

      // 6. Build Control Register 2:
      // Bit 3: Cmm_en = 1 (0x08)
      // Bits 2:0: CM_Freq (1 to 7)
      uint8_t cm_freq_bits = (config.outputRate & 0x07);
      uint8_t ctrl2_val = 0x08 | cm_freq_bits;

      // Bit 7: En_prd_set (0x80), Bits 6:4: Prd_set (convert 1-8 -> 0-7)
      if (config.setFrequency > 0) {
            uint8_t prd_set_bits = ((config.setFrequency - 1) & 0x07) << 4;
            ctrl2_val |= (0x80 | prd_set_bits);
      }
      write_reg(MMC_CTRL2, ctrl2_val);

      // 7. Enable Auto SET/RESET and Measurement Done Interrupt in Control Register 0 (0x09)
      write_reg(MMC_CTRL0, 0x24); 

      Serial.printf("[MMC] Configured continuous mode.\n");

      // Echo state of config by reading control registers
      MMC_Config_t out_config; // Holder of read back config values
      read_config(out_config);

      Serial.println("[MMC] MMC ready to be deployed\n");

      return true;
}

/**
 * \brief Unpacks the DMA buffer and applies 18-to-16 bit conversion.
 */
bool MMC5983MA::single_read() {
      spi_transaction_t *r_trans;
      
      if (spi_device_get_trans_result(spi_dma.handle, &r_trans, 0) != ESP_OK) {
            return false;
      }

      dma_in_progress = false;

      // The MMC5983MA outputs 18-bit values for X, Y, Z. Data is packed across bytes 1-7 in DMA buffer.
      uint32_t x_18 = ((uint32_t)spi_dma.rx_buffer[1] << 10) | ((uint32_t)spi_dma.rx_buffer[2] << 2);
      uint32_t y_18 = ((uint32_t)spi_dma.rx_buffer[3] << 10) | ((uint32_t)spi_dma.rx_buffer[4] << 2);
      uint32_t z_18 = ((uint32_t)spi_dma.rx_buffer[5] << 10) | ((uint32_t)spi_dma.rx_buffer[6] << 2);

      uint8_t extra_bits = spi_dma.rx_buffer[7];
      x_18 |= (extra_bits >> 6) & 0x03;
      y_18 |= (extra_bits >> 4) & 0x03;
      z_18 |= (extra_bits >> 2) & 0x03;

      //Check if same as last measurement
      if ((x_18 == mag_last_measurement[0]) && (y_18 == mag_last_measurement[1]) && (z_18 == mag_last_measurement[2])){
            //Measurements are exactly identical, no new measurement was made
            return false;
      }

      mag_last_measurement[0] = x_18;
      mag_last_measurement[1] = y_18;
      mag_last_measurement[2] = z_18;

      // Convert raw 18-bit values to Gauss
      data_holder.mx = ((float)x_18 - NULL_FIELD_18) / COUNTS_PER_G_18;
      data_holder.my = -1.0 * ((float)y_18 - NULL_FIELD_18) / COUNTS_PER_G_18; //Flipping the y axis to correct to right hand coordinate system
      data_holder.mz = ((float)z_18 - NULL_FIELD_18) / COUNTS_PER_G_18;

      Apply_Cal_Matrix();

      return true;
}

void MMC5983MA::Apply_Cal_Matrix() {
      // Subtract hard-iron centroid offset
      float dx = data_holder.mx - data_holder.offset[0];
      float dy = data_holder.my - data_holder.offset[1];
      float dz = data_holder.mz - data_holder.offset[2];

      // Multiply by soft-iron correction tensor W
      data_holder.mx_cal = data_holder.W[0][0] * dx + data_holder.W[0][1] * dy + data_holder.W[0][2] * dz;
      data_holder.my_cal = data_holder.W[1][0] * dx + data_holder.W[1][1] * dy + data_holder.W[1][2] * dz;
      data_holder.mz_cal = data_holder.W[2][0] * dx + data_holder.W[2][1] * dy + data_holder.W[2][2] * dz;
}

// --------------------------------------------------------------------------
// Calibration Math / Helper Functions (Private Members)
// --------------------------------------------------------------------------

bool MMC5983MA::Solve9x9(float A[9][9], float b[9], float x[9]) {
      for (int i = 0; i < 9; ++i) {
            int max_row = i;
            float max_val = fabsf(A[i][i]);
            for (int k = i + 1; k < 9; ++k) {
                  if (fabsf(A[k][i]) > max_val) {
                  max_val = fabsf(A[k][i]);
                  max_row = k;
                  }
            }
            if (max_val < 1e-7f) return false; 

            if (max_row != i) {
                  for (int k = i; k < 9; ++k) {
                  float tmp = A[i][k]; A[i][k] = A[max_row][k]; A[max_row][k] = tmp;
                  }
                  float tmp_b = b[i]; b[i] = b[max_row]; b[max_row] = tmp_b;
            }

            for (int k = i + 1; k < 9; ++k) {
                  float factor = A[k][i] / A[i][i];
                  for (int j = i; j < 9; ++j) A[k][j] -= factor * A[i][j];
                  b[k] -= factor * b[i];
            }
      }

      for (int i = 8; i >= 0; --i) {
            float sum = b[i];
            for (int j = i + 1; j < 9; ++j) sum -= A[i][j] * x[j];
            x[i] = sum / A[i][i];
      }
      return true;
      }

      bool MMC5983MA::Invert3x3(const float A[3][3], float inv[3][3]) {
      float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                  A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                  A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

      if (fabsf(det) < 1e-7f) return false;
      float inv_det = 1.0f / det;

      inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
      inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
      inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
      inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
      inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
      inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
      inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
      inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
      inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;
      return true;
}

bool MMC5983MA::MatrixSqrt3x3(const float M[3][3], float W[3][3]) {
      float Y[3][3], Z[3][3];
      memcpy(Y, M, sizeof(Y));
      memset(Z, 0, sizeof(Z));
      Z[0][0] = 1.0f; Z[1][1] = 1.0f; Z[2][2] = 1.0f;

      for (int iter = 0; iter < 12; ++iter) {
            float invY[3][3], invZ[3][3];
            if (!Invert3x3(Y, invY) || !Invert3x3(Z, invZ)) return false;

            float nextY[3][3], nextZ[3][3];
            for (int r = 0; r < 3; ++r) {
                  for (int c = 0; c < 3; ++c) {
                  nextY[r][c] = 0.5f * (Y[r][c] + invZ[r][c]);
                  nextZ[r][c] = 0.5f * (Z[r][c] + invY[r][c]);
                  }
            }
            memcpy(Y, nextY, sizeof(Y));
            memcpy(Z, nextZ, sizeof(Z));
      }

      memcpy(W, Y, sizeof(Y));
      return true;
}

size_t MMC5983MA::MMC_Read_Block(uint8_t block_seconds, std::vector<float>& x_out, std::vector<float>& y_out, std::vector<float>& z_out) {
      if (data_holder.reportFrequency <= 0) return 0;

      size_t target_samples = (size_t)block_seconds * data_holder.reportFrequency;
      size_t samples_read = 0;

      while (samples_read < target_samples) {
            if (new_data_ready) {
                  new_data_ready = false;
                  if (single_read()) {
                        x_out.push_back(data_holder.mx);
                        y_out.push_back(data_holder.my);
                        z_out.push_back(data_holder.mz);
                        samples_read++;
                  }
            }
      }
      return samples_read;
}

bool MMC5983MA::Evaluate_Calibration_Quality(const std::vector<float>& raw_x, const std::vector<float>& raw_y, const std::vector<float>& raw_z, const float offset[3], const float W[3][3]) {
      size_t n = raw_x.size();
      if (n < 60) return false; 

      float min_x = raw_x[0], max_x = raw_x[0];
      float min_y = raw_y[0], max_y = raw_y[0];
      float min_z = raw_z[0], max_z = raw_z[0];

      for (size_t i = 1; i < n; ++i) {
            if (raw_x[i] < min_x) min_x = raw_x[i];
            if (raw_x[i] > max_x) max_x = raw_x[i];
            if (raw_y[i] < min_y) min_y = raw_y[i];
            if (raw_y[i] > max_y) max_y = raw_y[i];
            if (raw_z[i] < min_z) min_z = raw_z[i];
            if (raw_z[i] > max_z) max_z = raw_z[i];
      }

      const float MIN_AXIS_RANGE = 0.25f;
      if ((max_x - min_x) < MIN_AXIS_RANGE || (max_y - min_y) < MIN_AXIS_RANGE || (max_z - min_z) < MIN_AXIS_RANGE) {
            Serial.println("[MMC] Insufficient spatial distribution; rotate across all 3 axes.");
            return false;
      }

      float sum_r = 0.0f;
      std::vector<float> r_vals(n);

      for (size_t i = 0; i < n; ++i) {
            float dx = raw_x[i] - offset[0];
            float dy = raw_y[i] - offset[1];
            float dz = raw_z[i] - offset[2];

            float cx = W[0][0]*dx + W[0][1]*dy + W[0][2]*dz;
            float cy = W[1][0]*dx + W[1][1]*dy + W[1][2]*dz;
            float cz = W[2][0]*dx + W[2][1]*dz + W[2][2]*dz;

            float r = sqrtf(cx*cx + cy*cy + cz*cz);
            r_vals[i] = r;
            sum_r += r;
      }

      float mean_r = sum_r / (float)n;
      if (mean_r < 0.05f) return false;

      float var_sum = 0.0f;
      for (size_t i = 0; i < n; ++i) {
            float diff = r_vals[i] - mean_r;
            var_sum += diff * diff;
      }
      float std_r = sqrtf(var_sum / (float)n);
      float norm_error = std_r / mean_r;

      Serial.printf("[MMC] Fit Mean Radius: %.4f G, Relative Error: %.2f%%\n", mean_r, norm_error * 100.0f);
      return (norm_error < 0.10f);
}

/**
 * \brief Fits an arbitrary 3D ellipsoid to sampled magnetometer data.
 */
bool MMC5983MA::Calibrate_Full_Soft_Hard_Iron(uint8_t num_seconds, uint8_t num_timeout) {
      //Reset calibration
      data_holder.offset[0] = 0.0f;
      data_holder.offset[1] = 0.0f;
      data_holder.offset[2] = 0.0f;

      data_holder.W[0][0] = 1.0f; data_holder.W[0][1] = 0.0f; data_holder.W[0][2] = 0.0f;
      data_holder.W[1][0] = 0.0f; data_holder.W[1][1] = 1.0f; data_holder.W[1][2] = 0.0f;
      data_holder.W[2][0] = 0.0f; data_holder.W[2][1] = 0.0f; data_holder.W[2][2] = 1.0f;

      if (num_seconds == 0) num_seconds = 3;
      if (num_timeout < num_seconds) num_timeout = num_seconds;

      std::vector<float> all_x;
      std::vector<float> all_y;
      std::vector<float> all_z;

      uint32_t start_time = millis();
      uint32_t timeout_ms = (uint32_t)num_timeout * 1000;
      bool is_good_calibration = false;

      Serial.printf("[MMC] Beginning calibration: %ds blocks, timeout %ds...\n", num_seconds, num_timeout);

      while ((millis() - start_time) < timeout_ms) {
            Serial.printf("[MMC] Sampling %d second block...\n", num_seconds);
            MMC_Read_Block(num_seconds, all_x, all_y, all_z);

            size_t total_samples = all_x.size();
            if (total_samples < 30) continue;

            float AtA[9][9] = {};
            float Atb[9]    = {};

            for (size_t k = 0; k < total_samples; ++k) {
                  float x = all_x[k], y = all_y[k], z = all_z[k];
                  float row[9] = { x*x, y*y, z*z, 2.0f*x*y, 2.0f*x*z, 2.0f*y*z, 2.0f*x, 2.0f*y, 2.0f*z };
                  for (int i = 0; i < 9; ++i) {
                  for (int j = 0; j < 9; ++j) AtA[i][j] += row[i] * row[j];
                  Atb[i] += row[i];
                  }
            }

            float p[9];
            if (!Solve9x9(AtA, Atb, p)) continue;

            float A_mat[3][3] = { { p[0], p[3], p[4] }, { p[3], p[1], p[5] }, { p[4], p[5], p[2] } };
            float v_vec[3] = { p[6], p[7], p[8] };

            float invA[3][3];
            if (!Invert3x3(A_mat, invA)) continue;

            float cand_offset[3];
            cand_offset[0] = -(invA[0][0]*v_vec[0] + invA[0][1]*v_vec[1] + invA[0][2]*v_vec[2]);
            cand_offset[1] = -(invA[1][0]*v_vec[0] + invA[1][1]*v_vec[1] + invA[1][2]*v_vec[2]);
            cand_offset[2] = -(invA[2][0]*v_vec[0] + invA[2][1]*v_vec[1] + invA[2][2]*v_vec[2]);

            float Av[3] = {
                  A_mat[0][0]*cand_offset[0] + A_mat[0][1]*cand_offset[1] + A_mat[0][2]*cand_offset[2],
                  A_mat[1][0]*cand_offset[0] + A_mat[1][1]*cand_offset[1] + A_mat[1][2]*cand_offset[2],
                  A_mat[2][0]*cand_offset[0] + A_mat[2][1]*cand_offset[1] + A_mat[2][2]*cand_offset[2]
            };

            float d_scale = 1.0f + (cand_offset[0]*Av[0] + cand_offset[1]*Av[1] + cand_offset[2]*Av[2]);
            if (d_scale <= 0.0f) continue;

            float M[3][3];
            for (int r = 0; r < 3; ++r) {
                  for (int c = 0; c < 3; ++c) M[r][c] = A_mat[r][c] / d_scale;
            }

            float cand_W[3][3];
            if (!MatrixSqrt3x3(M, cand_W)) continue;

            if (Evaluate_Calibration_Quality(all_x, all_y, all_z, cand_offset, cand_W)) {
                  memcpy(data_holder.offset, cand_offset, sizeof(cand_offset));
                  memcpy(data_holder.W, cand_W, sizeof(cand_W));
                  is_good_calibration = true;
                  Serial.printf("[MMC] Calibration converged successfully with %u samples!\n", (unsigned)total_samples);
                  break;
            }
      }

      if (!is_good_calibration) Serial.println("[MMC-ERROR] Calibration timed out without reaching target quality.");
      return is_good_calibration;
}