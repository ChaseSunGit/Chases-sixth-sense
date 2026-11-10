/**
 * \file SixthSense_BLE.h
 * \brief Dedicated transport layer for NimBLE server and notifications.
 *
 * \authors   Chase Sun
 */

#ifndef SIXTHSENSE_BLE_H
#define SIXTHSENSE_BLE_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string>

// Encapsulated configuration parameters for RF and Connection tuning
struct BLE_Config_t {
      String device_name;
      esp_power_level_t tx_power_level;  // Output power level for power management
      uint16_t adv_interval_ms;          // Advertising interval (unconnected)
      uint16_t min_conn_interval_ms;     // Minimum connection interval
      uint16_t max_conn_interval_ms;     // Maximum connection interval
      uint16_t peripherial_latency;      // Number of intervals the peripheral can skip
      uint16_t supervision_timeout_ms;   // Timeout before dropping connection
};

class SixthSense_BLE {
public:
      SixthSense_BLE();

      void begin(const BLE_Config_t &config);

      void sendPayload(const std::string &payload);
      
      bool isConnected() const { return connected; }
      void setConnected(bool state) { connected = state; }
      const BLE_Config_t& getConfig() const { return config; }

private:
      BLE_Config_t ble_config;
      bool connected = false;
      NimBLECharacteristic* pTxCharacteristic = NULL;
      NimBLEServer* pServer = NULL;
};

#endif // SIXTHSENSE_BLE_H