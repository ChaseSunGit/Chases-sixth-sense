/**
 * \file SixthSense_IO.cpp
 * \brief Implementation of the lean communication and packing tasks.
 */

#include "SixthSense_IO.h"
#include <string.h>

SixthSense_IO::SixthSense_IO() {
      memset(can_network_imu_data, 0, sizeof(can_network_imu_data));
      memset(active_can_channel, 0, sizeof(active_can_channel)); // Initialize data holders in memory
}

bool SixthSense_IO::begin(const IO_Config_t &config) {
      io_config = config;
      active_can_channel[0] = true;//Mark itself as active

      // Create 1-slot mailbox queue for zero-latency, fresh-only reads
      if (imu_data_queue == NULL) {
            imu_data_queue = xQueueCreate(1, sizeof(Fusion_Data_t)); //Only 1 deep as mailbox, new data always overwrite old data
            if (imu_data_queue == NULL) {
                  Serial.println("[IO] Failed to create Mailbox Queue");
                  return false;
            }
      }

      twai_general_config_t twai_g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TWAI_TX, CAN_TWAI_RX, TWAI_MODE_NORMAL);
      if ( io_config.mode == NODE_CENTRAL ){
            twai_g_config.rx_queue_len = 512; //Massive rx buffer that can handle 10+ packets of full received data at 8bytes * 5 peripheral sensors
      } else {
            twai_g_config.rx_queue_len = 8; //Don't need anything for rx, but assign 8 because that is the minimum size requirement
      }

      twai_g_config.tx_queue_len = 32; // max 8 data packs transmitted per cycle

      twai_filter_config_t twai_f_config;

      if (io_config.mode == NODE_PERIPHERAL) {
            // Peripheral Mode: Reject all inbound frames in hardware to conserve CPU cycles
            twai_f_config.single_filter = true;
            twai_f_config.acceptance_code = 0x00000000;
            twai_f_config.acceptance_mask = 0x00000000; 
      } else {
            // Central Mode: Hardware filter for IDs 0 - 127
            twai_f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
      }


      twai_timing_config_t twai_t_config;
      if (io_config.CAN_1MHZ_en){
            twai_t_config = TWAI_TIMING_CONFIG_1MBITS();
      } else {
            twai_t_config = TWAI_TIMING_CONFIG_500KBITS();
      }

      if (twai_driver_install(&twai_g_config, &twai_t_config, &twai_f_config) != ESP_OK) {
            Serial.println("[IO] Failed to install TWAI driver");
            return false;
      }

      if (twai_start() != ESP_OK) {
            Serial.println("[IO] Failed to start TWAI driver");
            return false;
      }

      Serial.println("[IO] Low-latency TWAI interface ready.");
      return true;
}

bool SixthSense_IO::RTOS_startIOTasks(BaseType_t core_id, UBaseType_t priority) {
      // Return early if tasks are already active
      if (tx_task_handle != NULL || rx_task_handle != NULL) {
            return true;
      }

      BaseType_t tx_res = pdPASS;
      BaseType_t rx_res = pdPASS;

      // The TX task handles emptying the local IMU queue and executing UART prints. 
      // It must be launched in ALL modes. (CAN transmission is safely gated inside the task).
      tx_res = xTaskCreatePinnedToCore(
            TX_Task_Trampoline, 
            "IO_TX_Task", 
            4096, 
            this, 
            priority, 
            &tx_task_handle, 
            core_id
      );

      // Only Central Mode needs to actively listen to the CAN bus for peripheral data
      if (io_config.mode == NODE_CENTRAL) {
            rx_res = xTaskCreatePinnedToCore(
                  RX_Task_Trampoline, 
                  "IO_RX_Task", 
                  4096, 
                  this, 
                  priority + 1, 
                  &rx_task_handle, 
                  core_id
            );
      }

      return (tx_res == pdPASS && rx_res == pdPASS);
}

void SixthSense_IO::RTOS_stopIOTasks() {
      if (tx_task_handle != NULL) {
            vTaskDelete(tx_task_handle);
            tx_task_handle = NULL;
      }
      if (rx_task_handle != NULL) {
            vTaskDelete(rx_task_handle);
            rx_task_handle = NULL;
      }
      twai_stop();
      twai_driver_uninstall();
}

void SixthSense_IO::TX_Task_Trampoline(void *pvParameters) {
      static_cast<SixthSense_IO*>(pvParameters)->transmitTask();
}

void SixthSense_IO::RX_Task_Trampoline(void *pvParameters) {
      static_cast<SixthSense_IO*>(pvParameters)->receiveTask();
}

void SixthSense_IO::transmitTask() {

      Fusion_Data_t local_imu_data;

      while (1) {
            // Blocks until Core 1 writes a new frame into the mailbox queue
            if (xQueueReceive(imu_data_queue, &local_imu_data, portMAX_DELAY) == pdTRUE) {
                  can_network_imu_data[0] = local_imu_data;
                  uint16_t base = io_config.local_can_id;

                  // Check what the device mode is configured to be
                  if ( io_config.mode == NODE_CENTRAL ) {
                        printUARTData();
                  } else if ( io_config.mode == NODE_PERIPHERAL ) {
                        // Transmit configured CAN messages
                        if (io_config.write_accelGyro) {
                              sendCANMessage(base + 1, local_imu_data.accel[0], local_imu_data.gyro[0]);
                              sendCANMessage(base + 2, local_imu_data.accel[1], local_imu_data.gyro[1]);
                              sendCANMessage(base + 3, local_imu_data.accel[2], local_imu_data.gyro[2]);
                        }
                        if (io_config.write_mag) {
                              sendCANMessage(base + 4, local_imu_data.mag[0], local_imu_data.mag[1]);
                        }
                        if (io_config.write_mag || io_config.write_euler) { 
                              sendCANMessage(base + 5, local_imu_data.mag[2], local_imu_data.yaw);
                        }
                        if (io_config.write_euler) {
                              sendCANMessage(base + 6, local_imu_data.roll, local_imu_data.pitch);
                        }
                        if (io_config.write_quat) { 
                              sendCANMessage(base + 7, local_imu_data.q[0], local_imu_data.q[1]);
                              sendCANMessage(base + 8, local_imu_data.q[2], local_imu_data.q[3]);
                        }
                        if (io_config.verbose_uart){
                              printUARTData_Debug();
                        }
                  } else {
                        Serial.println("IO Mode selection invalid!");
                  }
            }
      }
}

void SixthSense_IO::receiveTask() {
      twai_message_t rx_msg;

      while (1) {
            if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
                  if (rx_msg.data_length_code != 8) {
                        continue; //We only accept full 8 byte data because we know that everything else is not relevant
                  }

                  uint16_t base_id = (uint16_t)(rx_msg.identifier - (rx_msg.identifier % 10));
                  uint8_t msg_type = (uint8_t)(rx_msg.identifier % 10);

                  int p_idx = getPeripheralIndex(base_id); // Check if this is a device we are configured to listen to
                  if (p_idx == -1) continue;

                  active_can_channel[p_idx] = true; // Set the channel to active

                  float f1, f2;
                  memcpy(&f1, &rx_msg.data[0], sizeof(float));
                  memcpy(&f2, &rx_msg.data[4], sizeof(float));

                  switch (msg_type) {
                  case 1: can_network_imu_data[p_idx].accel[0] = f1; can_network_imu_data[p_idx].gyro[0] = f2; break;
                  case 2: can_network_imu_data[p_idx].accel[1] = f1; can_network_imu_data[p_idx].gyro[1] = f2; break;
                  case 3: can_network_imu_data[p_idx].accel[2] = f1; can_network_imu_data[p_idx].gyro[2] = f2; break;
                  case 4: can_network_imu_data[p_idx].mag[0]   = f1; can_network_imu_data[p_idx].mag[1]  = f2; break;
                  case 5: can_network_imu_data[p_idx].mag[2]   = f1; can_network_imu_data[p_idx].yaw     = f2; break;
                  case 6: can_network_imu_data[p_idx].roll     = f1; can_network_imu_data[p_idx].pitch   = f2; break;
                  case 7: can_network_imu_data[p_idx].q[0]     = f1; can_network_imu_data[p_idx].q[1]    = f2; break;
                  case 8: can_network_imu_data[p_idx].q[2]     = f1; can_network_imu_data[p_idx].q[3]    = f2; break;
                  default: break;
                  }
            }
      }
}

inline void SixthSense_IO::sendCANMessage(uint16_t msg_id, float f1, float f2) {
      // Lean transmission: 11-bit standard frame, 0 timeout (drop if TX buffer full)
      twai_message_t tx_msg;
      tx_msg.identifier = msg_id;
      tx_msg.data_length_code = 8;
      tx_msg.extd = 0;
      tx_msg.rtr = 0;

      memcpy(&tx_msg.data[0], &f1, sizeof(float));
      memcpy(&tx_msg.data[4], &f2, sizeof(float));

      twai_transmit(&tx_msg, 0);
      //twai_transmit(&tx_msg, pdMS_TO_TICKS(1));//Wait up to 3ms before timeout
}

int SixthSense_IO::getPeripheralIndex(uint16_t base_id) {
      for (int i = 0; i < 5; i++) {
            if (io_config.peripheral_can_ids[i] == base_id) return i + 1;
      }
      return -1;
}

void SixthSense_IO::printUARTData() {
      uint32_t current_ms = millis();
      Serial.printf("%u",current_ms);
      for (int i = 0; i < 6; i++) {
            if (!active_can_channel[i]) continue;

            switch (i) {
            case 0: Serial.printf(",PS"); break;
            case 1: Serial.printf(",LT"); break;
            case 2: Serial.printf(",LS"); break;
            case 3: Serial.printf(",RT"); break;
            case 4: Serial.printf(",RS"); break;
            case 5: Serial.printf(",BC"); break;
            default: break;
            }
            
            Fusion_Data_t &d = can_network_imu_data[i];

            if (io_config.write_accelGyro) {
                  Serial.printf(",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", d.accel[0]*GRAVITY_ICM,d.accel[1]*GRAVITY_ICM,d.accel[2]*GRAVITY_ICM,d.gyro[0]*DEG2RAD_ICM,d.gyro[1]*DEG2RAD_ICM,d.gyro[2]*DEG2RAD_ICM);
            }
            if (io_config.write_mag) {
                  Serial.printf(",%.4f,%.4f,%.4f", d.mag[0], d.mag[1], d.mag[2]);
            }
            if (io_config.write_euler) {
                  Serial.printf(",%.4f,%.4f,%.4f", d.roll, d.pitch, d.yaw);
            }
            if (io_config.write_quat) {
                  Serial.printf(",%.4f,%.4f,%.4f,%.4f", d.q[0], d.q[1], d.q[2], d.q[3]);
            }
            Serial.println();
      }
}

void SixthSense_IO::printUARTData_Debug() {
      uint32_t current_ms = millis();
      uint32_t dt = current_ms - lastTime_millis;
      lastTime_millis = current_ms;

      Fusion_Data_t &d = can_network_imu_data[0];
      Serial.printf("dt: %u, ms: %u\t", dt, current_ms);

      if (io_config.write_accelGyro) {
            Serial.printf("Accel:\t%.4f,%.4f,%.4f\tGyro:\t%.4f,%.4f,%.4f\t", d.accel[0]*GRAVITY_ICM,d.accel[1]*GRAVITY_ICM,d.accel[2]*GRAVITY_ICM,d.gyro[0]*DEG2RAD_ICM,d.gyro[1]*DEG2RAD_ICM,d.gyro[2]*DEG2RAD_ICM);
      }
      if (io_config.write_mag) {
            Serial.printf("Mag:\t%.4f,%.4f,%.4f\t", d.mag[0], d.mag[1], d.mag[2]);
      }
      if (io_config.write_euler) {
            Serial.printf("Euler:\t%.4f,%.4f,%.4f\t", d.roll, d.pitch, d.yaw);
      }
      if (io_config.write_quat) {
            Serial.printf("Quatornians\t%.4f,%.4f,%.4f,%.4f", d.q[0], d.q[1], d.q[2], d.q[3]);
      }
      Serial.println();

      
      // Non-blocking memory check
      //Serial.printf("[MEM] Free Heap: %u bytes | Min Free Ever: %u bytes | Largest Free Block: %u bytes\n",esp_get_free_heap_size(), esp_get_minimum_free_heap_size(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
      
}

void SixthSense_IO::returnDefaultConfig(IO_Config_t &io_config) {
      Serial.println("[Config] IO default config:");
      io_config = {
            .local_can_id = 100,    // Must be multiples of 10
            .peripheral_can_ids = {110, 120, 130, 140, 150}, // Also must be multiples of 10
            .mode = NODE_CENTRAL,                  // 0-2:    0: Central mode, no CAN broadcasting, gather CAN messages to send to UART
                                                            //1: Peripheral mode, no CAN reads, only CAN broadcasting
                                                            //2: BLE mode, yet to be implemented
            .CAN_1MHZ_en = 0,       //Enable CAN 1MHz, default disabled, 500kHz
            .write_accelGyro = 1,   //Enable accel + gyro write, default enabled
            .write_mag = 0,         //Enable magnetometer write, default disabled
            .write_euler = 0,       //Enable euler write, default disabled
            .write_quat = 0,        //Enable quatornian write, default disabled
            .verbose_uart = 1       //Print debug messages when set to peripheral mode, default enabled
      };
}