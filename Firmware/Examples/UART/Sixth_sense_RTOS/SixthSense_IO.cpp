/**
 * \file SixthSense_IO.cpp
 * \brief Implementation of communication, binary disassembly, SoC reading, and LED tasks.
 *
 * \author    Chase Sun
 */

#include "SixthSense_IO.h"
#include <string.h>

SixthSense_IO::SixthSense_IO() {
      memset(network_imu_data, 0, sizeof(network_imu_data));
      memset(network_soc_data, 0, sizeof(network_soc_data));
      memset(network_active_channel, 0, sizeof(network_active_channel)); // set all channels to inactive
}

bool SixthSense_IO::begin(const IO_Config_t &config, const Wireless_Config_t &wireless_config) {
      io_config = config;
      if (io_config.num_peripherals > MAX_PERIPHERAL_NODES) {
            io_config.num_peripherals = MAX_PERIPHERAL_NODES;
      }

      // Reset all network state so stale channels/data do not persist across re-inits
      memset(network_imu_data, 0, sizeof(network_imu_data));
      memset(network_soc_data, 0, sizeof(network_soc_data));
      memset(network_active_channel, 0, sizeof(network_active_channel));

      if (io_config.listener_mode){
            network_active_channel[0] = false; // Listen only, do not report local data
      } else {
            network_active_channel[0] = true; // Mark local sensor as active
      }
      
      //Check for wireless mode
      pinMode(PIN_WIRELESS_SET, INPUT);
      wireless_mode_enabled = digitalRead(PIN_WIRELESS_SET); // high means wireless is set

      // Configure Status LED
      pinMode(PIN_STATUS_LED, OUTPUT);
      digitalWrite(PIN_STATUS_LED, HIGH); // Default solid ON before transmission starts

      if (imu_data_queue == NULL) {
            imu_data_queue = xQueueCreate(1, sizeof(Fusion_Data_t));
            if (imu_data_queue == NULL) {
                  Serial.println("[IO] Failed to create Mailbox Queue");
                  return false;
            }
      }

      if (wireless_mode_enabled) {
            Serial.println("[IO] WIRELESS (ESP-NOW) mode.");
            return wireless.begin(
                  wireless_config,
                  (io_config.mode == NODE_CENTRAL),
                  io_config.peripheral_can_ids,
                  io_config.num_peripherals
            );
      } 
      else {
            Serial.println("[IO] WIRED (TWAI CAN) mode.");

            twai_general_config_t twai_g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TWAI_TX, CAN_TWAI_RX, TWAI_MODE_NORMAL);
            if (io_config.mode == NODE_CENTRAL) {
                  twai_g_config.rx_queue_len = 512;
            } else {
                  twai_g_config.rx_queue_len = 8;
            }
            twai_g_config.tx_queue_len = 32;
            twai_g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL;

            twai_filter_config_t twai_f_config;
            if (io_config.mode == NODE_PERIPHERAL) {
                  twai_f_config.single_filter = true;
                  twai_f_config.acceptance_code = 0xFFFFFFFF;
                  twai_f_config.acceptance_mask = 0x00000000;
            } else {
                  twai_f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
            }

            twai_timing_config_t twai_t_config;
            if (io_config.CAN_1MHZ_en) {
                  twai_t_config = TWAI_TIMING_CONFIG_1MBITS();
            } else {
                  twai_t_config = TWAI_TIMING_CONFIG_500KBITS();
                  twai_t_config.sjw = 4;
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
}

bool SixthSense_IO::RTOS_startIOTasks(BaseType_t core_id, UBaseType_t priority) {
      if (tx_task_handle != NULL || rx_task_handle != NULL || led_task_handle != NULL) {
            return true;
      }

      BaseType_t tx_res = pdPASS;
      BaseType_t rx_res = pdPASS;
      BaseType_t led_res = pdPASS;

      tx_res = xTaskCreatePinnedToCore(
            TX_Task_Trampoline,
            "IO_TX_Task",
            4096,
            this,
            priority,
            &tx_task_handle,
            core_id
      );

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

      led_res = xTaskCreatePinnedToCore(
            LED_Task_Trampoline,
            "IO_LED_Task",
            2048,
            this,
            1, // Low priority so it never preempts 200Hz telemetry
            &led_task_handle,
            1 // pin led task to core 1 always
      );

      return (tx_res == pdPASS && rx_res == pdPASS && led_res == pdPASS);
}

void SixthSense_IO::RTOS_stopIOTasks() {
      // Prevent new 200 Hz frames from starting and allow any in-flight esp_now_send() to finish
      tx_paused = true;
      vTaskDelay(pdMS_TO_TICKS(10));

      if (tx_task_handle != NULL) {
            vTaskDelete(tx_task_handle);
            tx_task_handle = NULL;
      }
      if (rx_task_handle != NULL) {
            vTaskDelete(rx_task_handle);
            rx_task_handle = NULL;
      }
      if (led_task_handle != NULL) {
            vTaskDelete(led_task_handle);
            led_task_handle = NULL;
      }

      if (PIN_STATUS_LED >= 0) {
            digitalWrite(PIN_STATUS_LED, HIGH); // Stay ON when stopped/paused
      }

      if (wireless_mode_enabled) {
            wireless.stop();
      } else {
            twai_stop();
            twai_driver_uninstall();
      }

      tx_paused = false; // Reset flag for when RTOS_startIOTasks() is called on exit
}

void SixthSense_IO::TX_Task_Trampoline(void *pvParameters) {
      static_cast<SixthSense_IO*>(pvParameters)->transmitTask();
}

void SixthSense_IO::RX_Task_Trampoline(void *pvParameters) {
      static_cast<SixthSense_IO*>(pvParameters)->receiveTask();
}

void SixthSense_IO::LED_Task_Trampoline(void *pvParameters) {
      static_cast<SixthSense_IO*>(pvParameters)->ledBlinkTask();
}

void SixthSense_IO::transmitTask() {
      Fusion_Data_t local_imu_data;

      while (1) {
            if (xQueueReceive(imu_data_queue, &local_imu_data, portMAX_DELAY) == pdTRUE) {
                  if (tx_paused) {
                        continue;
                  }

                  // Mark data transfer heartbeat for LED task
                  last_tx_activity_ms = millis();

                  network_imu_data[0] = local_imu_data;
                  network_imu_data[0].msTimeStamp = millis();//Get current timestamp
                  network_soc_data[0] = wireless.getSoCVoltage();
                  uint16_t base = io_config.local_can_id;

                  if (io_config.mode == NODE_CENTRAL) {
                        //Before sending data over uart, mark timed out network peripherals inactive
                        uint32_t current_millis = millis();
                        for (int i = 0; i <= io_config.num_peripherals; i++){
                              if (network_active_channel[i] && io_config.peripheral_timeout != 0){
                                    // if channel is active and if a non zero timeout is set
                                    // mark the channel as inactive if last read is longer than timeout
                                    if ( (current_millis - network_imu_data[i].msTimeStamp) > io_config.peripheral_timeout) {
                                          network_active_channel[i] = false;
                                    }
                              }
                        }
                        if (io_config.verbose_uart) {
                              printUARTData();
                        } else {
                              printBinaryData();
                        }
                  } else if (io_config.mode == NODE_PERIPHERAL) {
                        if (wireless_mode_enabled) {
                              // Low-latency single-packet broadcast at 200Hz
                              wireless.sendSensorPacket(
                                    base,
                                    local_imu_data,
                                    network_soc_data[0],
                                    io_config.write_accelGyro,
                                    io_config.write_mag,
                                    io_config.write_euler,
                                    io_config.write_quat
                              );
                        } else {
                              // Wired CAN transmission
                              if (io_config.write_accelGyro) {
                                    sendCANMessage(base + 0, local_imu_data.accel[0], local_imu_data.gyro[0]);
                                    sendCANMessage(base + 1, local_imu_data.accel[1], local_imu_data.gyro[1]);
                                    sendCANMessage(base + 2, local_imu_data.accel[2], local_imu_data.gyro[2]);
                              }
                              if (io_config.write_mag) {
                                    sendCANMessage(base + 3, local_imu_data.mag[0], local_imu_data.mag[1]);
                              }
                              if (io_config.write_mag || io_config.write_euler) {
                                    sendCANMessage(base + 4, local_imu_data.mag[2], local_imu_data.yaw);
                              }
                              if (io_config.write_euler) {
                                    sendCANMessage(base + 5, local_imu_data.roll, local_imu_data.pitch);
                              }
                              if (io_config.write_quat) {
                                    sendCANMessage(base + 6, local_imu_data.q[0], local_imu_data.q[1]);
                                    sendCANMessage(base + 7, local_imu_data.q[2], local_imu_data.q[3]);
                              }
                        }

                        if (io_config.verbose_uart) {
                              printUARTData_Debug();
                        }
                  }
            }
      }
}

void SixthSense_IO::receiveTask() {
      if (wireless_mode_enabled) {
            Wireless_Packet_t rx_pkt;
            QueueHandle_t w_queue = wireless.getRxQueue();

            while (1) {
                  if (xQueueReceive(w_queue, &rx_pkt, pdMS_TO_TICKS(50)) == pdTRUE) {
                        disassembleWirelessData(rx_pkt.data, rx_pkt.len, rx_pkt.mac);
                  }
            }
      } else {
            twai_message_t rx_msg;

            while (1) {
                  uint32_t alerts = 0;
                  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
                        if (alerts & TWAI_ALERT_BUS_OFF) {
                              Serial.println("[IO] Central Bus-Off! Recovering...");
                              twai_initiate_recovery();
                        }
                        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                              twai_start();
                        }
                        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                              twai_clear_receive_queue();
                        }
                  }

                  if (twai_receive(&rx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
                        if (rx_msg.extd || rx_msg.rtr || rx_msg.data_length_code != 8) {
                              continue;
                        }

                        uint16_t base_id = (uint16_t)(rx_msg.identifier - (rx_msg.identifier % 10));
                        uint8_t msg_type = (uint8_t)(rx_msg.identifier % 10);

                        int p_idx = getPeripheralIndex(base_id);
                        if (p_idx == -1) continue;

                        network_active_channel[p_idx] = true;
                        network_soc_data[p_idx] = 0.0f; // Always 0 during wired operation

                        float f1, f2;
                        memcpy(&f1, &rx_msg.data[0], sizeof(float));
                        memcpy(&f2, &rx_msg.data[4], sizeof(float));

                        // Get timestamp of reception of network data for timeout tracking
                        network_imu_data[p_idx].msTimeStamp = millis();

                        switch (msg_type) {
                        case 0: network_imu_data[p_idx].accel[0] = f1; network_imu_data[p_idx].gyro[0] = f2; break;
                        case 1: network_imu_data[p_idx].accel[1] = f1; network_imu_data[p_idx].gyro[1] = f2; break;
                        case 2: network_imu_data[p_idx].accel[2] = f1; network_imu_data[p_idx].gyro[2] = f2; break;
                        case 3: network_imu_data[p_idx].mag[0]   = f1; network_imu_data[p_idx].mag[1]  = f2; break;
                        case 4: network_imu_data[p_idx].mag[2]   = f1; network_imu_data[p_idx].yaw     = f2; break;
                        case 5: network_imu_data[p_idx].roll     = f1; network_imu_data[p_idx].pitch   = f2; break;
                        case 6: network_imu_data[p_idx].q[0]     = f1; network_imu_data[p_idx].q[1]    = f2; break;
                        case 7: network_imu_data[p_idx].q[2]     = f1; network_imu_data[p_idx].q[3]    = f2; break;
                        default: break;
                        }
                  }
            }
      }
}

void SixthSense_IO::ledBlinkTask() {
      // Wired = 5 Hz (1000ms period), Wireless = 1 Hz (200ms period)
      const uint32_t period_ms = wireless_mode_enabled ? 400 : 2000;
      const uint32_t on_time_ms = period_ms / 10;         // 10% of the period
      const uint32_t off_time_ms = period_ms - on_time_ms; // 90% of the period

      bool led_state = true;
      uint32_t last_toggle_ms = millis();

      while (1) {
            uint32_t now = millis();
            // Consider data transfer active if not explicitly paused AND a frame was transmitted within the last 50ms
            bool transfer_active = (!tx_paused) && ((now - last_tx_activity_ms) < 50) && (last_tx_activity_ms > 0);

            if (!transfer_active) {
                  // When data transfer is paused or idle, pause blinking and hold LED solid ON
                  led_state = true;
                  digitalWrite(PIN_STATUS_LED, LOW);
                  last_toggle_ms = now; // Constantly reset the timer so it starts with a clean ON cycle when active
            } else {
                  // Determine how long we should wait based on whether the LED is currently ON or OFF
                  uint32_t current_interval = led_state ? on_time_ms : off_time_ms;

                  if ((now - last_toggle_ms) >= current_interval) {
                        led_state = !led_state;
                        digitalWrite(PIN_STATUS_LED, led_state ? LOW : HIGH);
                        last_toggle_ms = now;
                  }
            }

            vTaskDelay(pdMS_TO_TICKS(10));
      }
}

inline void SixthSense_IO::sendCANMessage(uint16_t msg_id, float f1, float f2) {
      uint32_t alerts = 0;
      if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            if (alerts & TWAI_ALERT_BUS_OFF) {
                  Serial.println("[IO] Peripheral Bus-Off detected! Initiating recovery...");
                  twai_initiate_recovery();
                  return;
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                  twai_start();
                  Serial.println("[IO] Peripheral Bus Recovered and Restarted.");
            }
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                  twai_clear_receive_queue();
            }
      }

      twai_message_t tx_msg = {};
      tx_msg.identifier = msg_id;
      tx_msg.data_length_code = 8;
      tx_msg.extd = 0;
      tx_msg.rtr = 0;

      memcpy(&tx_msg.data[0], &f1, sizeof(float));
      memcpy(&tx_msg.data[4], &f2, sizeof(float));

      twai_transmit(&tx_msg, 0);
}

int SixthSense_IO::getPeripheralIndex(uint16_t base_id) {
      for (int i = 0; i < io_config.num_peripherals; i++) {
            if (io_config.peripheral_can_ids[i] == base_id) return i + 1;
      }
      return -1;
}

void SixthSense_IO::printUARTData() {
      const char* node_labels[TOTAL_NETWORK_NODES] = {
            "PL", "LT", "LS", "RT", "RS", "BC",
            "S6", "S7", "S8", "S9"
      };

      for (int i = 0; i <= io_config.num_peripherals; i++) {
            if (!network_active_channel[i]) continue; // Skip if network channel is not active

            Serial.print(node_labels[i]);

            Fusion_Data_t &d = network_imu_data[i];

            if (io_config.write_accelGyro) {
                  Serial.printf(",%.4f,%.4f,%.4f,%.6f,%.6f,%.6f",
                                d.accel[0] * GRAVITY_ICM, d.accel[1] * GRAVITY_ICM, d.accel[2] * GRAVITY_ICM,
                                d.gyro[0] * DEG2RAD_ICM, d.gyro[1] * DEG2RAD_ICM, d.gyro[2] * DEG2RAD_ICM);
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
            // Output SoC voltage after Quat
            if (wireless_mode_enabled) {
                  Serial.printf(",%d", network_soc_data[i]);
            }
            Serial.println();
      }
}

void SixthSense_IO::printUARTData_Debug() {
      uint32_t current_ms = millis();
      uint32_t dt = current_ms - lastTime_millis;
      lastTime_millis = current_ms;

      Fusion_Data_t &d = network_imu_data[0];
      Serial.printf("dt: %u, ms: %u\t", dt, current_ms);

      if (io_config.write_accelGyro) {
            Serial.printf("Accel:\t%.4f,%.4f,%.4f\tGyro:\t%.4f,%.4f,%.4f\t",
                          d.accel[0] * GRAVITY_ICM, d.accel[1] * GRAVITY_ICM, d.accel[2] * GRAVITY_ICM,
                          d.gyro[0] * DEG2RAD_ICM, d.gyro[1] * DEG2RAD_ICM, d.gyro[2] * DEG2RAD_ICM);
      }
      if (io_config.write_mag) {
            Serial.printf("Mag:\t%.4f,%.4f,%.4f\t", d.mag[0], d.mag[1], d.mag[2]);
      }
      if (io_config.write_euler) {
            Serial.printf("Euler:\t%.4f,%.4f,%.4f\t", d.roll, d.pitch, d.yaw);
      }
      if (io_config.write_quat) {
            Serial.printf("Quat:\t%.4f,%.4f,%.4f,%.4f\t", d.q[0], d.q[1], d.q[2], d.q[3]);
      }
      Serial.println();
}

uint16_t SixthSense_IO::assembleBinaryData(uint8_t* target_buf, size_t max_len) {
      // Minimum frame: Magic(2) + Info(1) + Timestamp(4) + SensorID(2) + SoC(2) + Checksum(1) = 12 bytes
      if (!target_buf || max_len < 12) {
            return 0;
      }

      uint16_t offset = 0;

      // 1. Calculate active sensors (Central + up to 14 peripherals = max 15)
      uint8_t active_count = 0;
      for (int i = 0; i <= io_config.num_peripherals; i++) {
            if (network_active_channel[i]) active_count++;
      }

      // 2. Build Info Byte
      uint8_t flags = 0;
      if (io_config.write_accelGyro) flags |= (1 << 0);
      if (io_config.write_mag)       flags |= (1 << 1);
      if (io_config.write_euler)     flags |= (1 << 2);
      if (io_config.write_quat)      flags |= (1 << 3);

      uint8_t info_byte = (active_count << 4) | (flags & 0x0F);

      // 3. Write Magic Word (0xFD, 0xAC), Info Byte, and Timestamp
      target_buf[offset++] = 0xFD;
      target_buf[offset++] = 0xAC;
      target_buf[offset++] = info_byte;

      uint32_t current_ms = millis();
      memcpy(&target_buf[offset], &current_ms, sizeof(current_ms));
      offset += sizeof(current_ms);

      // 4. Output sensor data sequentially
      for (int i = 0; i <= io_config.num_peripherals; i++) {
            if (!network_active_channel[i]) continue;

            uint16_t sensor_id = (i == 0) ? io_config.local_can_id : io_config.peripheral_can_ids[i - 1];
            memcpy(&target_buf[offset], &sensor_id, sizeof(sensor_id));
            offset += sizeof(sensor_id);

            Fusion_Data_t &d = network_imu_data[i];

            if (io_config.write_accelGyro) {
                  float ag[6] = {
                        (float)(d.accel[0] * GRAVITY_ICM),
                        (float)(d.accel[1] * GRAVITY_ICM),
                        (float)(d.accel[2] * GRAVITY_ICM),
                        (float)(d.gyro[0] * DEG2RAD_ICM),
                        (float)(d.gyro[1] * DEG2RAD_ICM),
                        (float)(d.gyro[2] * DEG2RAD_ICM)
                  };
                  memcpy(&target_buf[offset], ag, sizeof(ag));
                  offset += sizeof(ag);
            }

            if (io_config.write_mag) {
                  float m[3] = { (float)d.mag[0], (float)d.mag[1], (float)d.mag[2] };
                  memcpy(&target_buf[offset], m, sizeof(m));
                  offset += sizeof(m);
            }

            if (io_config.write_euler) {
                  float eu[3] = { (float)d.roll, (float)d.pitch, (float)d.yaw };
                  memcpy(&target_buf[offset], eu, sizeof(eu));
                  offset += sizeof(eu);
            }

            if (io_config.write_quat) {
                  float q[4] = { (float)d.q[0], (float)d.q[1], (float)d.q[2], (float)d.q[3] };
                  memcpy(&target_buf[offset], q, sizeof(q));
                  offset += sizeof(q);
            }

            // State of Charge Voltage (16bit int showing voltage in mV)
            uint16_t soc = network_soc_data[i];
            memcpy(&target_buf[offset], &soc, sizeof(soc));
            offset += sizeof(soc);
      }

      // 5. Calculate XOR Checksum (excluding 2-byte magic word)
      uint8_t checksum = 0;
      for (uint16_t j = 2; j < offset; j++) {
            checksum ^= target_buf[j];
      }
      target_buf[offset++] = checksum;

      return offset;
}

bool SixthSense_IO::disassembleWirelessData(const uint8_t* src_buf, uint16_t len, const uint8_t* sender_mac) {
      // Minimum frame: Magic(2) + Info(1) + Timestamp(4) + SensorID(2) + SoC(2) + Checksum(1) = 12 bytes
      if (!src_buf || len < 12) return false;

      // 1. Check Magic Word
      if (src_buf[0] != 0xBC || src_buf[1] != 0xDF) return false;

      // 2. Verify XOR Checksum
      uint8_t calc_checksum = 0;
      for (uint16_t j = 2; j < len - 1; j++) {
            calc_checksum ^= src_buf[j];
      }
      if (calc_checksum != src_buf[len - 1]) return false;

      uint16_t offset = 2;
      uint8_t info_byte = src_buf[offset++];
      uint8_t active_count = (info_byte >> 4) & 0x0F;
      uint8_t flags = info_byte & 0x0F;

      bool has_ag    = (flags & (1 << 0)) != 0;
      bool has_mag   = (flags & (1 << 1)) != 0;
      bool has_euler = (flags & (1 << 2)) != 0;
      bool has_quat  = (flags & (1 << 3)) != 0;

      // Skip 4-byte peripheral timestamp
      offset += sizeof(uint32_t);

      // 3. Unpack sensor blocks and save into local network array
      for (uint8_t s = 0; s < active_count; s++) {
            if (offset + sizeof(uint16_t) > len - 1) return false;

            uint16_t sensor_id = 0;
            memcpy(&sensor_id, &src_buf[offset], sizeof(sensor_id));
            offset += sizeof(sensor_id);

            // Check if sensor_id is in the Central's preconfigured peripheral list
            int p_idx = getPeripheralIndex(sensor_id);
            if (p_idx == -1) return false;

            // If received over wireless, verify/pair peer dynamically up to max_peripherals
            if (wireless_mode_enabled && sender_mac != nullptr) {
                  if (!wireless.verifyAndRegisterPeer(sensor_id, sender_mac)) {
                        return false;
                  }
            }

            Fusion_Data_t &d = network_imu_data[p_idx];

            // Note: assembleBinaryData scaled accel by GRAVITY_ICM and gyro by DEG2RAD_ICM.
            // Invert scaling on unpack so printUARTData() and assembleBinaryData() don't double-scale!
            if (has_ag) {
                  if (offset + 6 * sizeof(float) > len - 1) return false;
                  float ag[6];
                  memcpy(ag, &src_buf[offset], sizeof(ag));
                  offset += sizeof(ag);

                  d.accel[0] = ag[0] / GRAVITY_ICM;
                  d.accel[1] = ag[1] / GRAVITY_ICM;
                  d.accel[2] = ag[2] / GRAVITY_ICM;
                  d.gyro[0]  = ag[3] / DEG2RAD_ICM;
                  d.gyro[1]  = ag[4] / DEG2RAD_ICM;
                  d.gyro[2]  = ag[5] / DEG2RAD_ICM;
            }

            if (has_mag) {
                  if (offset + 3 * sizeof(float) > len - 1) return false;
                  float m[3];
                  memcpy(m, &src_buf[offset], sizeof(m));
                  offset += sizeof(m);

                  d.mag[0] = m[0];
                  d.mag[1] = m[1];
                  d.mag[2] = m[2];
            }

            if (has_euler) {
                  if (offset + 3 * sizeof(float) > len - 1) return false;
                  float eu[3];
                  memcpy(eu, &src_buf[offset], sizeof(eu));
                  offset += sizeof(eu);

                  d.roll  = eu[0];
                  d.pitch = eu[1];
                  d.yaw   = eu[2];
            }

            if (has_quat) {
                  if (offset + 4 * sizeof(float) > len - 1) return false;
                  float q[4];
                  memcpy(q, &src_buf[offset], sizeof(q));
                  offset += sizeof(q);

                  d.q[0] = q[0];
                  d.q[1] = q[1];
                  d.q[2] = q[2];
                  d.q[3] = q[3];
            }

            // Read State of Charge int
            if (offset + sizeof(uint16_t) > len - 1) return false;
            uint16_t soc = 0;
            memcpy(&soc, &src_buf[offset], sizeof(soc));
            offset += sizeof(soc);

            if (wireless_mode_enabled){
                  network_soc_data[p_idx] = soc;
            }
            else{
                  network_soc_data[p_idx] = 0;
            }
            d.msTimeStamp = millis();
            network_active_channel[p_idx] = true;
      }

      return true;
}

void SixthSense_IO::printBinaryData() {
      // Max size for 10 sensors * 70 bytes + 8 bytes overhead = 708 bytes
      uint8_t tx_buf[768];
      uint16_t len = assembleBinaryData(tx_buf, sizeof(tx_buf));

      if (len > 0) {
            Serial.write(tx_buf, len);
      }
}

void SixthSense_IO::returnDefaultConfig(IO_Config_t &io_config) {
      Serial.println("[Config] IO default config:");
      io_config = {
            .local_can_id = 100,
            .peripheral_can_ids = {110, 120, 130, 140, 150, 160, 170, 180, 190},
            .peripheral_timeout = 100, // default timeout after 100 ms
            .num_peripherals = 4,     // Default 4 peripherals (configurable up to 9)
            .mode = NODE_CENTRAL,
            .CAN_1MHZ_en = 1,
            .write_accelGyro = 1,
            .write_mag = 0,
            .write_euler = 0,
            .write_quat = 0,
            .verbose_uart = 1,
            .listener_mode = 0
      };
}