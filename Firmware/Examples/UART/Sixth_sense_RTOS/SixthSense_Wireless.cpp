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

void SixthSense_Wireless::initRadioOnly() {
      WiFi.persistent(false);
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();

      uint8_t protocol_bitmap = WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
      if (config.enable_long_range_mode) protocol_bitmap |= (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_LR);
      esp_wifi_set_protocol(WIFI_IF_STA, protocol_bitmap);
      esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
      esp_wifi_set_channel(config.wifi_channel, WIFI_SECOND_CHAN_NONE);

      if (esp_now_init() != ESP_OK) {
            Serial.println("[Wireless] ESP-NOW init failed");
            return;
      }

      esp_wifi_set_max_tx_power(config.tx_power_quarter_dbm);

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

      if (central_mode) {
            esp_wifi_set_ps(WIFI_PS_NONE);
            esp_now_register_recv_cb(onDataRecv);
      } else {
            tx_submitted = 0;
            tx_completed = 0;
            last_cb_ms = millis();
            dbg_last_cb_ms = last_cb_ms;
            esp_now_register_send_cb(onDataSent);

            esp_wifi_set_ps(WIFI_PS_NONE);
            //esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
            //esp_wifi_connectionless_module_set_wake_interval(65000); 
            //esp_now_set_wake_window(1);

            esp_now_unregister_recv_cb();
      }
}

void SixthSense_Wireless::reinitEspNowOnly() {
      static uint32_t last_reset_ms = 0;
      uint32_t now = millis();
      if (now - last_reset_ms < 500) {
            return; // Cooldown prevents tight reset loops
      }
      last_reset_ms = now;

      Serial.printf("[Wireless-WDG] Resetting Wi-Fi/ESP-NOW (age=%lums)...\n", now - last_cb_ms);

      // Exact same teardown as SixthSense_Wireless::stop() (which we know 100% recovers the link)
      esp_wifi_set_ps(WIFI_PS_NONE);
      esp_now_unregister_send_cb();
      esp_now_deinit();
      WiFi.mode(WIFI_OFF);
      vTaskDelay(pdMS_TO_TICKS(5));

      initRadioOnly();
      recovery_count++;
}

bool SixthSense_Wireless::begin(const Wireless_Config_t &w_config, bool is_central, const uint16_t *allowed_ids, uint8_t num_allowed_ids) {
      config = w_config;
      central_mode = is_central;
      allowed_id_count = num_allowed_ids;
      paired_peer_count = 0;
      wifi_debug = w_config.enable_wireless_debug;

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

      initRadioOnly();

      if (!is_central) {
            if (soc_task_handle == NULL) {
                  pinMode(PIN_SOC_ADC, INPUT);
                  analogReadResolution(12);
                  analogSetPinAttenuation(PIN_SOC_ADC, ADC_11db);

                  cached_soc_voltage = (uint16_t)analogReadMilliVolts(PIN_SOC_ADC);
                  cached_soc_percent = computeStateOfCharge(cached_soc_voltage);

                  xTaskCreatePinnedToCore(SoC_Task_Trampoline, "Wireless_SoC", 2048, this, 1, &soc_task_handle, 1);
            }

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

      esp_wifi_set_ps(WIFI_PS_NONE);

      esp_now_unregister_send_cb();
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
      (void)analogReadMilliVolts(PIN_SOC_ADC);
      uint16_t filtered_mv = (uint16_t)analogReadMilliVolts(PIN_SOC_ADC);
      cached_soc_voltage = filtered_mv;
      cached_soc_percent = computeStateOfCharge(cached_soc_voltage);

      while (1) {
            vTaskDelay(pdMS_TO_TICKS(1013));

            (void)analogReadMilliVolts(PIN_SOC_ADC);
            uint16_t samples[5];
            for (int i = 0; i < 5; i++) {
                  samples[i] = (uint16_t)analogReadMilliVolts(PIN_SOC_ADC);
                  vTaskDelay(pdMS_TO_TICKS(2));
            }

            for (int i = 1; i < 5; i++) {
                  uint16_t key = samples[i];
                  int j = i - 1;
                  while (j >= 0 && samples[j] > key) {
                        samples[j + 1] = samples[j];
                        j--;
                  }
                  samples[j + 1] = key;
            }
            uint16_t median_mv = samples[2];

            if (median_mv >= 1200 && median_mv <= 2500) {
                  filtered_mv = (uint16_t)((filtered_mv * 3 + median_mv) / 4);
                  cached_soc_voltage = filtered_mv;
                  cached_soc_percent = computeStateOfCharge(cached_soc_voltage);
            }
      }
}

uint16_t SixthSense_Wireless::assembleSingleSensorBinary(uint8_t *target_buf, size_t max_len,
                                                         uint16_t sensor_id, const Fusion_Data_t &d, uint16_t soc_voltage,
                                                         bool write_ag, bool write_mag, bool write_euler, bool write_quat) {
      if (!target_buf || max_len < 12) {
            return 0;
      }

      uint16_t offset = 0;

      uint8_t active_count = 1;
      uint8_t flags = 0;
      if (write_ag)    flags |= (1 << 0);
      if (write_mag)   flags |= (1 << 1);
      if (write_euler) flags |= (1 << 2);
      if (write_quat)  flags |= (1 << 3);

      uint8_t info_byte = (active_count << 4) | (flags & 0x0F);

      target_buf[offset++] = 0xBC;
      target_buf[offset++] = 0xDF;
      target_buf[offset++] = info_byte;

      uint32_t current_ms = millis();
      memcpy(&target_buf[offset], &current_ms, sizeof(current_ms));
      offset += sizeof(current_ms);

      memcpy(&target_buf[offset], &sensor_id, sizeof(sensor_id));
      offset += sizeof(sensor_id);

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

      if (write_mag) {
            float m[3] = { (float)d.mag[0], (float)d.mag[1], (float)d.mag[2] };
            memcpy(&target_buf[offset], m, sizeof(m));
            offset += sizeof(m);
      }

      if (write_euler) {
            float eu[3] = { (float)d.roll, (float)d.pitch, (float)d.yaw };
            memcpy(&target_buf[offset], eu, sizeof(eu));
            offset += sizeof(eu);
      }

      if (write_quat) {
            float q[4] = { (float)d.q[0], (float)d.q[1], (float)d.q[2], (float)d.q[3] };
            memcpy(&target_buf[offset], q, sizeof(q));
            offset += sizeof(q);
      }

      memcpy(&target_buf[offset], &soc_voltage, sizeof(soc_voltage));
      offset += sizeof(soc_voltage);

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

      uint32_t now_ms = millis();

      // If MAC callback has been completely dead for >100 ms, reset Wi-Fi/ESP-NOW
      // Note: Do NOT return early here—fall through so we immediately transmit on the fresh radio!
      if (tx_submitted > 10 && (now_ms - last_cb_ms) > 100) {
            reinitEspNowOnly();
            now_ms = millis();
      }

      dbg_tx_attempts++;
      tx_submitted++;
      esp_err_t err = esp_now_send(BROADCAST_MAC, tx_buf, len);
      if (err != ESP_OK) {
            dbg_tx_api_err++;
            dbg_last_err = err;

            if (err == ESP_ERR_ESPNOW_NO_MEM) {
                  reinitEspNowOnly();
            }
      }

      if (wifi_debug){
      // 1 Hz diagnostic summary
            static uint32_t last_dbg_ms = 0;
            if (now_ms - last_dbg_ms >= 1000) {
                  uint32_t age = (now_ms >= last_cb_ms) ? (now_ms - last_cb_ms) : 0;
                  Serial.printf("[P-DBG t=%lus] att:%lu | err:%lu (0x%X) | cb_ok:%lu | age:%lums | rec:%lu\n",
                        now_ms / 1000, dbg_tx_attempts, dbg_tx_api_err, dbg_last_err,
                        dbg_cb_ok, age, recovery_count);

                  dbg_tx_attempts = 0;
                  dbg_tx_api_err = 0;
                  dbg_cb_ok = 0;
                  last_dbg_ms = now_ms;
            }
      }

      return (err == ESP_OK);
}

bool SixthSense_Wireless::verifyAndRegisterPeer(uint16_t sensor_can_id, const uint8_t *mac_addr) {
      for (uint8_t i = 0; i < allowed_id_count; i++) {
            if (discovered_peers[i].can_id == sensor_can_id) {
                  if (discovered_peers[i].paired) {
                        if (memcmp(discovered_peers[i].mac, mac_addr, 6) == 0) {
                              discovered_peers[i].last_seen_ms = millis();
                              return true;
                        }

                        if (millis() - discovered_peers[i].last_seen_ms > 1000) {
                              memcpy(discovered_peers[i].mac, mac_addr, 6);
                              discovered_peers[i].last_seen_ms = millis();
                              return true;
                        }

                        return false;
                  }

                  if (paired_peer_count >= allowed_id_count) {
                        return false;
                  }

                  memcpy(discovered_peers[i].mac, mac_addr, 6);
                  discovered_peers[i].paired = true;
                  discovered_peers[i].last_seen_ms = millis();
                  paired_peer_count++;
                  return true;
            }
      }
      return false;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void SixthSense_Wireless::onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
#else
void SixthSense_Wireless::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
      if (!instance) return;
      instance->tx_completed++;
      uint32_t now = millis();
      instance->last_cb_ms = now;
      instance->dbg_last_cb_ms = now;
      if (status == ESP_NOW_SEND_SUCCESS) {
            instance->dbg_cb_ok++;
      } else {
            instance->dbg_cb_fail++;
      }
}

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
            .wifi_channel = 11,
            .tx_power_quarter_dbm = 32,
            .phy_rate = WIFI_PHY_RATE_24M,
            .enable_long_range_mode = false,
            .enable_wireless_debug = false
      };
}

/**
     * Computes State of Charge (0-100) from a millivolt measurement.
     *
     * Rules:
     * - 1650 mV to 1900 mV maps linearly to 0 - 100.
     * - 1900 mV to 2000 mV outputs 200 (as specified).
     * - Below 1650 mV outputs 0.
     * - Hysteresis: SoC cannot increase unless voltage exceeds 2000 mV
     *   (indicating an external charge source).
     */
uint16_t SixthSense_Wireless::computeStateOfCharge(uint16_t voltage_mv) {
      // Track the lowest reported SoC; initialize to a value above max normal SoC (100)
      // so the very first reading establishes the baseline.
      static uint16_t latched_soc = 200;
      static bool is_charging = false;

      // 1. Check for external charge source (> 2000 mV)
      if (voltage_mv > 2000) {
            is_charging = true;
            latched_soc = 200;
            return 200;
      }

      // Once voltage drops below 1900 mV, clear the active charging latch
      is_charging = false;

      // 2. Check the 1900 mV to 2000 mV band
      if (voltage_mv >= 1920) {
            return 100;
      }

      // Once voltage drops below 1900 mV, clear the active charging latch
      

      // 3. Calculate raw linear SoC for 1650 mV - 1900 mV
      uint16_t raw_soc = 0;
      if (voltage_mv > 1650) {
            // Use uint32_t to prevent overflow during multiplication (250 * 100 = 25,000, fits in u16, 
            // but u32 is safer practice for fixed-point scaling).
            // Formula: ((voltage_mv - 1650) * 100) / (1900 - 1650)
            raw_soc = static_cast<uint16_t>((static_cast<uint32_t>(voltage_mv - 1650) * 100U) / 270U);
      }

      // 4. Enforce downward-only hysteresis while not charging
      if (raw_soc < latched_soc) {
            latched_soc = raw_soc;
      }

      return latched_soc;
}