/**
 * \file MMC5983MA_Driver.cpp
 * \brief Library for interfacing with MMC5983MA.
 *
 * \author Chase Sun
 */

#include "MMC5983MA_Driver.h"

// Initialze the global variables declared in the header
//Variables used for SPI communication
spi_device_handle_t spi_MMC = NULL;
SPI_DMA_Channel SPI_DMA_MMC = {};

//Flags for data ready
volatile bool dma_in_progress_MMC = false;
volatile bool new_mag_data_ready = false;
extern bool MMC_first_read;

//Data holding structure
MMC_Data MMC_Data_Holder = {};

//Initialize hardiron offset as zeros and soft iron matrix as I
MMC_Data_Holder.offset = {0.0,0.0,0.0};
MMC_Data_Holder.W = {
                        {1.0, 0.0, 0.0},
                        {0.0, 1.0, 0.0},
                        {0.0, 0.0, 1.0}
                    }

// ISR for data ready interrupt from the magnetometer
void IRAM_ATTR MMC_ISR_dataReady() {
      if (!dma_in_progress_MMC) {
            if (spi_device_queue_trans(spi_MMC, &SPI_DMA_MMC.trans, 0) == ESP_OK) {
                  dma_in_progress_MMC = true;
            }
      }
}

// ISR for DMA SPI Transaction complete
void IRAM_ATTR MMC_ISR_DMAcomplete_callback(spi_transaction_t *trans) {
      new_mag_data_ready = true;
}


/**
 * \brief Write to an MMC5983MA register. The command byte consists of a Write bit (0) and a 6-bit address.
 */
void MMC_write_reg(uint8_t reg, uint8_t data) {
      spi_transaction_t t = {};
      t.flags = SPI_TRANS_USE_TXDATA;
      t.length = 16; 
      t.tx_data[0] = reg & MMC_SPI_ADDR_MASK; // Bit 7 is 0 for write
      t.tx_data[1] = data;
      spi_device_polling_transmit(spi_MMC, &t);
}

/**
 * \brief Read an MMC5983MA register. The command byte consists of a Read bit (1) and a 6-bit address.
 */
uint8_t MMC_read_reg(uint8_t reg) {
      spi_transaction_t t = {};
      t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
      t.length = 16;
      t.tx_data[0] = (reg & MMC_SPI_ADDR_MASK) | MMC_SPI_READ_FLAG; 
      t.tx_data[1] = 0x00; 
      spi_device_polling_transmit(spi_MMC, &t);
      return t.rx_data[1];
}

/**
 * \brief Initializes the MMC5983MA into continuous measurement mode at a specified data rate.
 * 
 * \param outputRate Supported frequencies: 7 (1000 Hz), 6 (200 Hz), 5 (100 Hz), 4 (50 Hz), 3 (20 Hz), 2 (10 Hz), or 1 (1 Hz).
 * \return true if the sensor was found and configured successfully, false otherwise.
 */
bool MMC_init_chip(uint8_t outputRate) {
      //Zero out the data holder
      MMC_Data_Holder = {};

      //First setup the SPI bus

      // Initialize the SPI bus
      if (!SPI_Bus_Init(PIN_MOSI_MMC, PIN_MISO_MMC, PIN_SCLK_MMC)) {
            Serial.println("SPI host initialization failed!");
            return 0;
      }

      // Add the specific SPI device
      if (!SPI_Add_Device(PIN_CS_MMC, MMC_ISR_DMAcomplete_callback, SPI_DMA_MMC.handle)) {
            Serial.println("Could not add MMC Device!");
            return 0;
      }

      // Initialize the SPI bus
      if (!SPI_Arm_DMA_Channel(MMC_BURST_LEN, ((MMC_XOUT0 & MMC_SPI_ADDR_MASK) | MMC_SPI_READ_FLAG), SPI_DMA_MMC)) {
            Serial.println("DMA setup for MMC failed!");
            return 0;
      }

      // 1. Issue Software Reset to clear registers and reload OTP shadow registers
      MMC_write_reg(MMC_CTRL1, 0x80); // SW_RST bit
      delay(15);                      // Wait 10-15 ms for power-on/OTP reload cycle

      // 2. Verify Product ID (must read 0x30 / 48 dec)
      uint8_t prod_id = MMC_read_reg(MMC_PROD_ID);
      if (prod_id != 0x30) {
            Serial.printf("[MMC ERROR] Invalid Product ID: 0x%02X (Expected 0x30)\n", prod_id);
            return false;
      }
      Serial.printf("[MMC] Valid Product ID found: 0x%02X\n", prod_id);

      // 3. Perform an initial SET pulse to align AMR magnetic domains
      MMC_write_reg(MMC_CTRL0, 0x08); // Set bit = 1
      delay(1);                       // Brief settling time for CAP reservoir capacitor

      // 4. Determine register configurations for the requested rate
      uint8_t bw_setting = 0x00;   // Register 0x0A (Internal Control 1)
      uint8_t ctrl2_val  = 0x00;   // Register 0x0B (Internal Control 2)

      // Base flags for Control Register 2:
      // Bit 7: En_prd_set = 1 (enables automatic periodic SET to maintain polarization)
      // Bits 6:4: Prd_set = 001 (SET executed every 25 measurements)
      // Bit 3: Cmm_en = 1 (Continuous Measurement Mode Enabled)
      const uint8_t CTRL2_BASE = 0x98; // 0b10011000

      switch (outputRate) {
            case 7:
                  // 1000 Hz requires BW=11 (0.5 ms conversion time) and CM_Freq=111
                  bw_setting = 0x03; // BW = 11
                  ctrl2_val  = CTRL2_BASE | 0x07; // CM_Freq = 111
                  MMC_Data_Holder.reportFrequency = 1000;
                  break;

            case 6:
                  // 200 Hz requires BW=01 (4 ms conversion time) and CM_Freq=110
                  bw_setting = 0x01; // BW = 01
                  ctrl2_val  = CTRL2_BASE | 0x06; // CM_Freq = 110
                  MMC_Data_Holder.reportFrequency = 200;
                  break;

            case 5:
                  // 100 Hz uses default BW=00 (8 ms conversion time) and CM_Freq=101
                  bw_setting = 0x00; // BW = BW = 100Hz
                  ctrl2_val  = CTRL2_BASE | 0x05; // CM_Freq = 101
                  MMC_Data_Holder.reportFrequency = 100;
                  break;

            case 4:
                  // 50 Hz uses BW=00 and CM_Freq=100
                  bw_setting = 0x00; // BW = BW = 100Hz
                  ctrl2_val  = CTRL2_BASE | 0x04; // CM_Freq = 100
                  MMC_Data_Holder.reportFrequency = 50;
                  break;

            case 3:
                  // 20 Hz uses BW=00 and CM_Freq=011
                  bw_setting = 0x00; // BW = 100Hz
                  ctrl2_val  = CTRL2_BASE | 0x03; // CM_Freq = 011
                  MMC_Data_Holder.reportFrequency = 20;
                  break;

            case 2:
                  // 10 Hz uses BW=00 and CM_Freq=010
                  bw_setting = 0x00; // BW = BW = 100Hz
                  ctrl2_val  = CTRL2_BASE | 0x02; // CM_Freq = 010
                  MMC_Data_Holder.reportFrequency = 10;
                  break;

            case 1:
                  // 1 Hz uses BW=00 and CM_Freq=001
                  bw_setting = 0x00; // BW = 00
                  ctrl2_val  = CTRL2_BASE | 0x01; // CM_Freq = 001
                  MMC_Data_Holder.reportFrequency = 1;
                  break;

            default:
                  Serial.println("[MMC ERROR] Unsupported rate! Use 7 (1000 Hz), 6 (200 Hz), 5 (100 Hz), 4 (50 Hz), 3 (20 Hz), 2 (10 Hz), or 1 (1 Hz).");
                  return false;
      }

      // 5. Apply the Bandwidth setting to Internal Control 1 (0x0A)
      MMC_write_reg(MMC_CTRL1, bw_setting);

      // 6. Apply Continuous Mode Frequency and Periodic SET to Internal Control 2 (0x0B)
      MMC_write_reg(MMC_CTRL2, ctrl2_val);

      // 7. Enable Auto SET/RESET (Bit 5) and Measurement Done Interrupt (Bit 2) in Control Register 0 (0x09)
      // 0x24 = 0b00100100 (Auto_SR_en = 1, INT_meas_done_en = 1)
      MMC_write_reg(MMC_CTRL0, 0x24);

      Serial.printf("[MMC] Configured for continuous mode at %d Hz.\n", MMC_Data_Holder.reportFrequency);
      return true;
}

/**
 * \brief Unpacks the DMA buffer and applies 16-bit conversions per MMC5983MA_RevA_4-3-19.pdf.
 */
bool MMC_single_read() {
      
      spi_transaction_t *r_trans;
    
      // Check if DMA transfer completed
      if (spi_device_get_trans_result(spi_MMC, &r_trans, 0) != ESP_OK) {
            return false;
      }

      dma_in_progress_MMC = false;

      //The MMC5983MA outputs 18-bit values for X, Y, Z, and 8-bit for temperature. The data is packed across 9 bytes in the DMA buffer.
      //Bits 18-3 of each axis are in the first 6 bytes, and the last 2 bits of each axis are in byte 7. Temperature is in byte 8.
      //This can be read in 16 bit mode by ignoring the last 2 bits of each axis
      //We will skip temperature compensation for now and just read the raw values

      uint32_t x_18 = ((uint32_t)SPI_DMA_MMC.rx_buffer[1] << 10) | ((uint32_t)SPI_DMA_MMC.rx_buffer[2] << 2);
      uint32_t y_18 = ((uint32_t)SPI_DMA_MMC.rx_buffer[3] << 10) | ((uint32_t)SPI_DMA_MMC.rx_buffer[4] << 2);
      uint32_t z_18 = ((uint32_t)SPI_DMA_MMC.rx_buffer[5] << 10) | ((uint32_t)SPI_DMA_MMC.rx_buffer[6] << 2);

      uint8_t extra_bits = SPI_DMA_MMC.rx_buffer[7];
      x_18 |= (extra_bits >> 6) & 0x03;//Only keep the relevant 2 bits for each axis
      y_18 |= (extra_bits >> 4) & 0x03;
      z_18 |= (extra_bits >> 2) & 0x03;

      MMC_write_reg(MMC_STATUS, 0x01);

      // 4. Calculations to convert raw 18-bit values to Gauss. The datasheet specifies that the null field value (131072) should be subtracted from the raw value, and then divided by the counts per Gauss (16384) to get the field in Gauss.
      
      MMC_Data_Holder.mx = ((float)x_18 - NULL_FIELD_18) / COUNTS_PER_G_18;
      MMC_Data_Holder.my = ((float)y_18 - NULL_FIELD_18) / COUNTS_PER_G_18;
      MMC_Data_Holder.mz = ((float)z_18 - NULL_FIELD_18) / COUNTS_PER_G_18;

      return true;
}


//The following are math functions used by the soft iron calibration of the magnetometer

// Solves A * x = b via Gaussian elimination with partial pivoting (dimension 9)
static bool Solve9x9(float A[9][9], float b[9], float x[9]) {
      for (int i = 0; i < 9; ++i) {
            // Pivot selection
            int max_row = i;
            float max_val = fabsf(A[i][i]);
            for (int k = i + 1; k < 9; ++k) {
                  if (fabsf(A[k][i]) > max_val) {
                  max_val = fabsf(A[k][i]);
                  max_row = k;
                  }
            }
            if (max_val < 1e-7f) return false; // Singular system

            // Swap rows in A and b
            if (max_row != i) {
                  for (int k = i; k < 9; ++k) {
                  float tmp = A[i][k]; A[i][k] = A[max_row][k]; A[max_row][k] = tmp;
                  }
                  float tmp_b = b[i]; b[i] = b[max_row]; b[max_row] = tmp_b;
            }

            // Eliminate column entries below pivot
            for (int k = i + 1; k < 9; ++k) {
                  float factor = A[k][i] / A[i][i];
                  for (int j = i; j < 9; ++j) {
                  A[k][j] -= factor * A[i][j];
                  }
                  b[k] -= factor * b[i];
            }
      }

      // Back substitution
      for (int i = 8; i >= 0; --i) {
            float sum = b[i];
            for (int j = i + 1; j < 9; ++j) {
                  sum -= A[i][j] * x[j];
            }
            x[i] = sum / A[i][i];
      }
      return true;
}

// Inverts a 3x3 matrix. Returns false if determinant is near-zero.
static bool Invert3x3(const float A[3][3], float inv[3][3]) {
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

// Computes symmetric matrix square root W = sqrt(M) via Denman-Beavers iteration
static bool MatrixSqrt3x3(const float M[3][3], float W[3][3]) {
      float Y[3][3], Z[3][3];
      memcpy(Y, M, sizeof(Y));

      // Initialize Z as identity matrix
      memset(Z, 0, sizeof(Z));
      Z[0][0] = 1.0f; Z[1][1] = 1.0f; Z[2][2] = 1.0f;

      // Iteratively converge Y -> M^(1/2)
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

/**
 * \brief Reads a block of raw magnetometer data over the specified duration.
 * \param block_seconds Duration in seconds to collect data.
 * \param x_out Vector to append raw X samples.
 * \param y_out Vector to append raw Y samples.
 * \param z_out Vector to append raw Z samples.
 * \return Number of samples successfully gathered.
 */
static size_t MMC_Read_Block(uint8_t block_seconds, 
                             std::vector<float>& x_out, 
                             std::vector<float>& y_out, 
                             std::vector<float>& z_out) 
{
    if (MMC_Data_Holder.reportFrequency <= 0) return 0;

    size_t target_samples = (size_t)block_seconds * MMC_Data_Holder.reportFrequency;
    size_t samples_read = 0;
    uint32_t start_ms = millis();
    uint32_t timeout_ms = (uint32_t)block_seconds * 1000 + 500; // Extra margin

    while (samples_read < target_samples && (millis() - start_ms < timeout_ms)) {
        if (new_mag_data_ready) {
            new_mag_data_ready = false;
            if (MMC_single_read()) {
                x_out.push_back(MMC_Data_Holder.mx);
                y_out.push_back(MMC_Data_Holder.my);
                z_out.push_back(MMC_Data_Holder.mz);
                samples_read++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield CPU to watchdog / background tasks
    }
    return samples_read;
}

/**
 * \brief Evaluates whether the collected dataset has sufficient 3D coverage 
 *        and if the fitted calibration yields an approximately spherical field.
 */
static bool Evaluate_Calibration_Quality(const std::vector<float>& raw_x,
                                         const std::vector<float>& raw_y,
                                         const std::vector<float>& raw_z,
                                         const float offset[3],
                                         const float W[3][3]){

      size_t n = raw_x.size();
      if (n < 60) return false; // Minimum required points

      // 1. Check spatial span (reject 1D/2D degenerate sampling)
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

      // Require at least 0.25 Gauss variation across all axes
      const float MIN_AXIS_RANGE = 0.25f;
      if ((max_x - min_x) < MIN_AXIS_RANGE ||
            (max_y - min_y) < MIN_AXIS_RANGE ||
            (max_z - min_z) < MIN_AXIS_RANGE) {
            Serial.println("[CAL] Insufficient spatial distribution; rotate across all 3 axes.");
            return false;
      }

      // 2. Assess calibrated radius consistency (Normalized Standard Deviation)
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
      if (mean_r < 0.05f) return false; // Unrealistic mean magnetic field

      float var_sum = 0.0f;
      for (size_t i = 0; i < n; ++i) {
            float diff = r_vals[i] - mean_r;
            var_sum += diff * diff;
      }
      float std_r = sqrtf(var_sum / (float)n);
      float norm_error = std_r / mean_r;

      Serial.printf("[CAL] Fit Mean Radius: %.4f G, Relative Error: %.2f%%\n", mean_r, norm_error * 100.0f);

      // Good fit condition: standard deviation of calibrated field is within 10%
      return (norm_error < 0.10f);
}

/**
 * \brief Reads blocks of magnetometer data, computes calibration, assesses quality,
 *        and repeats until a good fit is found or timeout is reached.
 * \param num_seconds  Block sampling duration in seconds. Default = 3s
 * \param num_timeout  Maximum cumulative time to attempt calibration. Default = 30s
 * \return true on successful quality calibration, false on timeout.
 */
bool Calibrate_Full_Soft_Iron(uint8_t num_seconds, uint8_t num_timeout) {
      if (num_seconds == 0) num_seconds = 3;
      if (num_timeout < num_seconds) num_timeout = num_seconds;

      std::vector<float> all_x;
      std::vector<float> all_y;
      std::vector<float> all_z;

      uint32_t start_time = millis();
      uint32_t timeout_ms = (uint32_t)num_timeout * 1000;
      bool is_good_calibration = false;

      Serial.printf("[CAL] Beginning calibration: %ds blocks, timeout %ds...\n", num_seconds, num_timeout);

      while ((millis() - start_time) < timeout_ms) {
            Serial.printf("[CAL] Sampling %d second block...\n", num_seconds);
            MMC_Read_Block(num_seconds, all_x, all_y, all_z);

            size_t total_samples = all_x.size();
            if (total_samples < 30) {
                  continue;
            }

            // 1. Construct 9x9 normal equations: D^T * D * p = D^T * 1
            float AtA[9][9] = {};
            float Atb[9]    = {};

            for (size_t k = 0; k < total_samples; ++k) {
                  float x = all_x[k];
                  float y = all_y[k];
                  float z = all_z[k];

                  float row[9] = {
                  x * x,
                  y * y,
                  z * z,
                  2.0f * x * y,
                  2.0f * x * z,
                  2.0f * y * z,
                  2.0f * x,
                  2.0f * y,
                  2.0f * z
                  };

                  for (int i = 0; i < 9; ++i) {
                  for (int j = 0; j < 9; ++j) {
                        AtA[i][j] += row[i] * row[j];
                  }
                  Atb[i] += row[i];
                  }
            }

            // 2. Solve linear system
            float p[9];
            if (!Solve9x9(AtA, Atb, p)) {
                  Serial.println("[CAL] Matrix solve failed (singular). Continuing sampling...");
                  continue;
            }

            float A_mat[3][3] = {
                  { p[0], p[3], p[4] },
                  { p[3], p[1], p[5] },
                  { p[4], p[5], p[2] }
            };
            float v_vec[3] = { p[6], p[7], p[8] };

            // 3. Compute centroid & soft iron matrix
            float invA[3][3];
            if (!Invert3x3(A_mat, invA)) {
                  Serial.println("[CAL] Invert3x3 failed. Continuing sampling...");
                  continue;
            }

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
            if (d_scale <= 0.0f) {
                  Serial.println("[CAL] Degenerate ellipsoid (d_scale <= 0). Continuing sampling...");
                  continue;
            }

            float M[3][3];
            for (int r = 0; r < 3; ++r) {
                  for (int c = 0; c < 3; ++c) {
                  M[r][c] = A_mat[r][c] / d_scale;
                  }
            }

            float cand_W[3][3];
            if (!MatrixSqrt3x3(M, cand_W)) {
                  Serial.println("[CAL] MatrixSqrt3x3 failed. Continuing sampling...");
                  continue;
            }

            // 4. Validate fit quality
            if (Evaluate_Calibration_Quality(all_x, all_y, all_z, cand_offset, cand_W)) {
                  memcpy(MMC_Data_Holder.offset, cand_offset, sizeof(cand_offset));
                  memcpy(MMC_Data_Holder.W, cand_W, sizeof(cand_W));
                  is_good_calibration = true;
                  Serial.printf("[CAL] Calibration converged successfully with %u samples!\n", (unsigned)total_samples);
                  break;
            }

            Serial.println("[CAL] Fit quality not yet met. Accumulating more data...");
      }

      if (!is_good_calibration) {
            Serial.println("[CAL ERROR] Calibration timed out without reaching target quality.");
      }

      return is_good_calibration;
}

/**
 * \brief Fits an arbitrary 3D ellipsoid to sampled magnetometer data using least-squares
 *        and computes both the hard-iron centroid and full 3x3 soft-iron matrix W.
 *        If calibration fails due to insufficient or degenerate data, sampling time is
 *        extended incrementally up to a maximum of 30 seconds.
 * \param num_seconds     Initial sample duration in seconds.
 * \return true on successful fit and matrix decomposition, false if singular or degenerate after 30s.
 */
void Apply_Cal_Matrix() {
      // 1. Subtract hard-iron centroid offset
      float dx = in_x - MMC_Data_Holder.offset[0];
      float dy = in_y - MMC_Data_Holder.offset[1];
      float dz = in_z - MMC_Data_Holder.offset[2];

      // 2. Multiply by soft-iron correction tensor W
      out_x = MMC_Data_Holder.W[0][0] * dx + MMC_Data_Holder.W[0][1] * dy + MMC_Data_Holder.W[0][2] * dz;
      out_y = MMC_Data_Holder.W[1][0] * dx + MMC_Data_Holder.W[1][1] * dy + MMC_Data_Holder.W[1][2] * dz;
      out_z = MMC_Data_Holder.W[2][0] * dx + MMC_Data_Holder.W[2][1] * dy + MMC_Data_Holder.W[2][2] * dz;
}