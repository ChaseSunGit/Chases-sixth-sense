/**
 * \file SixthSense_Wireless.h
 * \brief Low-latency ESP-NOW broadcast communication handler for wearable IMUs.
 *
 * \author    Chase Sun
 */

#ifndef SIXTHSENSE_WIRELESS_H
#define SIXTHSENSE_WIRELESS_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_pm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <cstdint>
#include "SixthSense_IMU.h"

#define ESPNOW_MAX_PERIPHERALS 9
#define ESPNOW_MAX_PACKET_LEN  128

// Moved from GPIO_NUM_18 (ADC2 - shared with Wi-Fi) to GPIO_NUM_1 (ADC1 - isolated)
#define PIN_SOC_ADC       GPIO_NUM_18

// Raw packet wrapper passed from ESP-NOW ISR callback to IO RX task
struct Wireless_Packet_t {
      uint8_t mac[6];
      uint8_t data[ESPNOW_MAX_PACKET_LEN];
      uint16_t len;
};

// Track discovered/paired peripheral sensors on the Central node
struct Discovered_Peer_t {
      uint16_t can_id;
      uint8_t mac[6];
      bool paired;
      uint32_t last_seen_ms;
};

struct Wireless_Config_t {
      uint8_t wifi_channel;               // 1 - 11 (Pick 1, 6, or 11 to avoid Wi-Fi overlap)
      int8_t tx_power_quarter_dbm;        // 8 to 84 (84 = +21 dBm max power for toe-to-back body shadowing)
      wifi_phy_rate_t phy_rate;           // WIFI_PHY_RATE_24M (24 Mbps low airtime) or WIFI_PHY_RATE_6M (long range)
      bool enable_long_range_mode;
      bool enable_wireless_debug;
};

class SixthSense_Wireless {
public:
      SixthSense_Wireless();

      bool begin(const Wireless_Config_t &w_config, bool is_central, const uint16_t *allowed_ids, uint8_t num_allowed_ids);
      void stop();

      bool sendSensorPacket(uint16_t local_can_id, const Fusion_Data_t &imu_data, uint16_t soc_voltage,
                            bool write_ag, bool write_mag, bool write_euler, bool write_quat);

      uint16_t assembleSingleSensorBinary(uint8_t *target_buf, size_t max_len,
                                          uint16_t sensor_id, const Fusion_Data_t &d, uint16_t soc_voltage,
                                          bool write_ag, bool write_mag, bool write_euler, bool write_quat);

      QueueHandle_t getRxQueue() const { return rx_packet_queue; }
      bool verifyAndRegisterPeer(uint16_t sensor_can_id, const uint8_t *mac_addr);
      uint8_t getPairedCount() const { return paired_peer_count; }

      // Zero-latency getter for the transmitTask loop
      uint16_t getSoCVoltage() const { return cached_soc_voltage; }
      uint16_t getSoCPercent() const { return cached_soc_percent; }

      static void returnDefaultConfig(Wireless_Config_t &w_config);

private:
      Wireless_Config_t config;
      bool central_mode = false;
      bool wifi_debug = false;

      volatile uint16_t cached_soc_voltage = 0;
      volatile uint16_t cached_soc_percent = 0;
      TaskHandle_t soc_task_handle = NULL;

      uint8_t allowed_id_count = 0;

      Discovered_Peer_t discovered_peers[ESPNOW_MAX_PERIPHERALS];
      uint8_t paired_peer_count = 0;

      QueueHandle_t rx_packet_queue = NULL;
      static SixthSense_Wireless *instance;
      static const uint8_t BROADCAST_MAC[6];

      volatile uint32_t tx_submitted = 0;
      volatile uint32_t tx_completed = 0;
      volatile uint32_t last_cb_ms = 0;
      uint32_t recovery_count = 0;


      // Diagnostics
      volatile uint32_t dbg_tx_attempts = 0;
      volatile uint32_t dbg_tx_skipped = 0;
      volatile uint32_t dbg_tx_api_err = 0;
      volatile esp_err_t dbg_last_err = ESP_OK;
      volatile uint32_t dbg_cb_ok = 0;
      volatile uint32_t dbg_cb_fail = 0;
      volatile uint32_t dbg_last_cb_ms = 0;

      void initRadioOnly();
      static void Recovery_Task_Trampoline(void *pvParameters);
      void executeFastRecovery();
      void reinitEspNowOnly();

      static void SoC_Task_Trampoline(void *pvParameters);
      void socReadTask();

      static uint16_t computeStateOfCharge(uint16_t voltage_mv);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
      static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
#else
      static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
      static void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
#else
      static void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
#endif
};

#endif // SIXTHSENSE_WIRELESS_H