/**
 * \file SixthSense_Wireless.cpp
 * \brief Implementation of ESP-NOW wireless communication for wearable IMUs.
 *
 * \author    Chase Sun
 */

#include "SixthSense_Wireless.h"
#include <string.h>

SixthSense_Wireless* SixthSense_Wireless::instance = nullptr;
const uint8_t SixthSense_Wireless::BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

SixthSense_Wireless::SixthSense_Wireless() {
      memset(discovered_peers, 0, sizeof(discovered_peers));
      instance = this;
}

bool SixthSense_Wireless::begin(const Wireless_Config_t &w_config, bool is_central, const uint16_t *allowed_ids, uint8_t num_allowed_ids) {
      config = w_config;
      central_mode = is_central;
      
      allowed_id_count = num_allowed_ids;

      paired_peer_count = 0; // reset number of paired sensors

      for (uint8_t i = 0; i < allowed_id_count; i++) {
            discovered_peers[i].can_id = allowed_ids[i];
            discovered_peers[i].paired = false;
      }

      if (central_mode && rx_packet_queue == NULL) {
            rx_packet_queue = xQueueCreate(64, sizeof(Wireless_Packet_t));
            if (rx_packet_queue == NULL) {
                  Serial.println("[Wireless] Failed to create RX packet queue");
                  return false;
            }
      }

      // 1. Initialize Wi-Fi in Station Mode
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();

      // 2. Configure Protocol, Bandwidth & Channel (exclude 802.11b unless LR mode is enabled)
      uint8_t protocol_bitmap = WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
      if (config.enable_long_range_mode) protocol_bitmap |= (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR);
      esp_wifi_set_protocol(WIFI_IF_STA, protocol_bitmap);
      esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
      esp_wifi_set_channel(config.wifi_channel, WIFI_SECOND_CHAN_NONE);

      // 3. Initialize ESP-NOW
      if (esp_now_init() != ESP_OK) {
            Serial.println("[Wireless] ESP-NOW init failed");
            return false;
      }

      // 4. Set TX Power (after Wi-Fi/ESP-NOW start so PHY calibration doesn't override it)
      esp_wifi_set_max_tx_power(config.tx_power_quarter_dbm);

      // 5. Register Broadcast Peer & Configure PHY Rate
      esp_now_peer_info_t broadcast_peer = {};
      memcpy(broadcast_peer.peer_addr, BROADCAST_MAC, 6);
      broadcast_peer.channel = config.wifi_channel;
      broadcast_peer.ifidx = WIFI_IF_STA;
      broadcast_peer.encrypt = false;
      if (!esp_now_is_peer_exist(BROADCAST_MAC)) esp_now_add_peer(&broadcast_peer);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
      esp_now_rate_config_t rate_config = {
            .phymode = config.enable_long_range_mode ? WIFI_PHY_MODE_LR : WIFI_PHY_MODE_11G,
            .rate = config.phy_rate,
            .ersu = false,
            .dcm = false
      };
      esp_now_set_peer_rate_config(BROADCAST_MAC, &rate_config);
#else
      esp_wifi_config_espnow_rate(WIFI_IF_STA, config.phy_rate);
#endif

      // 6. Central vs Peripheral Power Save & Callbacks
      if (central_mode) {
            esp_wifi_set_ps(WIFI_PS_NONE);        // Zero-latency continuous RX on central
            esp_now_register_recv_cb(onDataRecv); // Called every time a new wireless packet is received
      } else {
            esp_err_t err1 = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);   // Sleep RF between 200 Hz TX bursts on TX-only peripherals
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
            esp_err_t err2 = esp_wifi_connectionless_module_set_wake_interval(100); 
            esp_err_t err3 = esp_now_set_wake_window(0); // 0 ms RX listen window (TX-only node)
            Serial.printf("[Wireless] PS err: %d | WakeInt err: %d | WakeWin err: %d\n", err1, err2, err3);
#endif
            esp_now_unregister_recv_cb();
      }

      // 7. Initialize SoC Hardware & Launch Low-Priority Reader Task if it is not the central node
      if (!is_central) {
            pinMode(PIN_SOC_ADC, INPUT);
            analogReadResolution(12);
            analogSetPinAttenuation(PIN_SOC_ADC, ADC_11db);
                  
            // Take immediate initial reading
            cached_soc_voltage = (uint16_t)analogReadMilliVolts(PIN_SOC_ADC);

            // Pin to Core 1 at Priority 1 so Core 0 stays dedicated to Wi-Fi/ESP-NOW
            xTaskCreatePinnedToCore(SoC_Task_Trampoline, "Wireless_SoC", 2048, this, 1, &soc_task_handle, 1);

            Serial.printf("[Wireless] ESP-NOW Ready (PERIPHERAL) | Ch: %u | TX Pwr: %.1f dBm\n", config.wifi_channel, config.tx_power_quarter_dbm * 0.25f);
      } else {
            Serial.printf("[Wireless] ESP-NOW Ready (CENTRAL) | Ch: %u | TX Pwr: %.1f dBm\n", config.wifi_channel, config.tx_power_quarter_dbm * 0.25f);
      }

      return true;
}

void SixthSense_Wireless::stop() {
      if (soc_task_handle != NULL) {
            vTaskDelete(soc_task_handle);
            soc_task_handle = NULL;
      }

      // Force RF domain awake before teardown so WiFi.mode(WIFI_OFF) does not hang in modem sleep
      esp_wifi_set_ps(WIFI_PS_NONE);

      esp_now_unregister_recv_cb();
      esp_now_deinit();
      WiFi.mode(WIFI_OFF);
      
      if (rx_packet_queue != NULL) {
            vQueueDelete(rx_packet_queue);
            rx_packet_queue = NULL;
      }
}

void SixthSense_Wireless::SoC_Task_Trampoline(void *pvParameters) {
      static_cast<SixthSense_Wireless*>(pvParameters)->socReadTask();
}

void SixthSense_Wireless::socReadTask() {
      while (1) {
            // Block and yield completely to the scheduler for 1 second
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            cached_soc_voltage = (uint16_t)analogReadMilliVolts(PIN_SOC_ADC); //read it as raw int mv
      }
}

uint16_t SixthSense_Wireless::assembleSingleSensorBinary(uint8_t *target_buf, size_t max_len,
                                                         uint16_t sensor_id, const Fusion_Data_t &d, uint16_t soc_voltage,
                                                         bool write_ag, bool write_mag, bool write_euler, bool write_quat) {

      // Minimum frame: Magic(2) + Info(1) + Timestamp(4) + SensorID(2) + SoC(2) + Checksum(1) = 12 bytes
      if (!target_buf || max_len < 12) {
            return 0;
      }

      uint16_t offset = 0;

      // Always 1 sensor for wireless peripheral frame
      uint8_t active_count = 1;
      uint8_t flags = 0;
      if (write_ag)    flags |= (1 << 0);
      if (write_mag)   flags |= (1 << 1);
      if (write_euler) flags |= (1 << 2);
      if (write_quat)  flags |= (1 << 3);

      uint8_t info_byte = (active_count << 4) | (flags & 0x0F);

      // Header: Magic Word (0xBC, 0xAF), Info Byte, Timestamp (4B)
      target_buf[offset++] = 0xBC;
      target_buf[offset++] = 0xDF;
      target_buf[offset++] = info_byte;

      uint32_t current_ms = millis();
      memcpy(&target_buf[offset], &current_ms, sizeof(current_ms));
      offset += sizeof(current_ms);

      // Sensor CAN ID (2B)
      memcpy(&target_buf[offset], &sensor_id, sizeof(sensor_id));
      offset += sizeof(sensor_id);

      // Accel + Gyro (24B)
      if (write_ag) {
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

      // Mag (12B)
      if (write_mag) {
            float m[3] = { (float)d.mag[0], (float)d.mag[1], (float)d.mag[2] };
            memcpy(&target_buf[offset], m, sizeof(m));
            offset += sizeof(m);
      }

      // Euler (12B)
      if (write_euler) {
            float eu[3] = { (float)d.roll, (float)d.pitch, (float)d.yaw };
            memcpy(&target_buf[offset], eu, sizeof(eu));
            offset += sizeof(eu);
      }

      // Quat (16B)
      if (write_quat) {
            float q[4] = { (float)d.q[0], (float)d.q[1], (float)d.q[2], (float)d.q[3] };
            memcpy(&target_buf[offset], q, sizeof(q));
            offset += sizeof(q);
      }

      // State of Charge Voltage (2B int, always write)
      memcpy(&target_buf[offset], &soc_voltage, sizeof(soc_voltage));
      offset += sizeof(soc_voltage);

      // XOR Checksum (excluding 2-byte magic word)
      uint8_t checksum = 0;
      for (uint16_t j = 2; j < offset; j++) {
            checksum ^= target_buf[j];
      }
      target_buf[offset++] = checksum;

      return offset;
}

bool SixthSense_Wireless::sendSensorPacket(uint16_t local_can_id, const Fusion_Data_t &imu_data, uint16_t soc_voltage,
                                           bool write_ag, bool write_mag, bool write_euler, bool write_quat) {
      uint8_t tx_buf[ESPNOW_MAX_PACKET_LEN];
      uint16_t len = assembleSingleSensorBinary(tx_buf, sizeof(tx_buf), local_can_id, imu_data, soc_voltage,
                                                write_ag, write_mag, write_euler, write_quat);
      if (len == 0) return false;

      return (esp_now_send(BROADCAST_MAC, tx_buf, len) == ESP_OK);
}

bool SixthSense_Wireless::verifyAndRegisterPeer(uint16_t sensor_can_id, const uint8_t *mac_addr) {
      for (uint8_t i = 0; i < allowed_id_count; i++) {
            if (discovered_peers[i].can_id == sensor_can_id) {
                  
                  // If already paired with a device
                  if (discovered_peers[i].paired) {
                        // If MAC matches, it's the valid sensor -> update timestamp & accept
                        if (memcmp(discovered_peers[i].mac, mac_addr, 6) == 0) { // compare incoming mac address to stored mac
                              discovered_peers[i].last_seen_ms = millis(); // millis are used for timeout tracking
                              return true;
                        }

                        // If MAC does NOT match:
                        // Only allow reassignment if the previous sensor timed out (> 1000 ms)
                        if (millis() - discovered_peers[i].last_seen_ms > 1000) {
                              memcpy(discovered_peers[i].mac, mac_addr, 6); // since connection timed out, save the new mac to correspond to the canid
                              // The reason for this timeout -> reassignment is to be able to replace broken sensors with new ones configured for the same can id during operation
                              discovered_peers[i].last_seen_ms = millis();
                              return true;
                        }

                        // Reject rogue/duplicate sensor with identical CAN ID
                        return false;
                  }

                  // Sensor id is a member of the allowed list, but it remains unpaired
                  // Check capacity limit
                  if (paired_peer_count >= allowed_id_count) {
                        return false; // Do not allow for additional pairing if max allowed pairs is exceeded
                  }

                  // Pair with the first MAC seen for this CAN ID
                  memcpy(discovered_peers[i].mac, mac_addr, 6);
                  discovered_peers[i].paired = true;
                  discovered_peers[i].last_seen_ms = millis();
                  paired_peer_count++;
                  return true;
            }
      }
      return false; // CAN ID not in preconfigured list
}


// Following are data receive call backs. Different procedures depending on different esp-idf version
// Functions simply take an incoming wifi packet and stores it into a queue
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void SixthSense_Wireless::onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
      if (!instance || !instance->rx_packet_queue || len <= 0 || len > ESPNOW_MAX_PACKET_LEN) return;

      Wireless_Packet_t pkt;
      memcpy(pkt.mac, recv_info->src_addr, 6);
      memcpy(pkt.data, incomingData, len);
      pkt.len = (uint16_t)len;

      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xQueueSendFromISR(instance->rx_packet_queue, &pkt, &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
      }
}
#else
void SixthSense_Wireless::onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
      if (!instance || !instance->rx_packet_queue || len <= 0 || len > ESPNOW_MAX_PACKET_LEN) return;

      Wireless_Packet_t pkt;
      memcpy(pkt.mac, mac_addr, 6);
      memcpy(pkt.data, incomingData, len);
      pkt.len = (uint16_t)len;

      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xQueueSendFromISR(instance->rx_packet_queue, &pkt, &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
      }
}
#endif

void SixthSense_Wireless::returnDefaultConfig(Wireless_Config_t &w_config) {
      w_config = {
            .wifi_channel = 11,                       // Channel 11
            .tx_power_quarter_dbm = 48,               // 48 * 0.25 = +12.0 dBm for ~2m wearable link
            .phy_rate = WIFI_PHY_RATE_24M,            // 24 Mbps OFDM: super short airtime for 200Hz real-time control
            .enable_long_range_mode = false           // Standard 802.11g/n for 24M rate
      };
}