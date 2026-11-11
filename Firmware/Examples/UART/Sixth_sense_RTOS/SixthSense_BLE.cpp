/**
 * \file SixthSense_BLE.cpp
 * \brief Implementation of the NimBLE transport layer.
 */

#include "SixthSense_BLE.h"

#define BLE_SERVICE_UUID           "532c0000-c9af-433b-846d-6ed179d4747e"
#define BLE_CHARACTERISTIC_UUID_TX "532c0001-c9af-433b-846d-6ed179d4747e"

class ServerCallbacks: public NimBLEServerCallbacks {
      SixthSense_BLE* ble_instance;

public:
      ServerCallbacks(SixthSense_BLE* instance) : ble_instance(instance) {}
      
      void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override { 
            ble_instance->setConnected(true); 
            
            // Convert milliseconds to BLE baseband time units
            uint16_t min_int = ble_instance->getConfig().min_conn_interval_ms / 1.25;
            uint16_t max_int = ble_instance->getConfig().max_conn_interval_ms / 1.25;
            uint16_t timeout = ble_instance->getConfig().supervision_timeout_ms / 10;
            
            pServer->updateConnParams(connInfo.getConnHandle(), min_int, max_int, 
                                    ble_instance->getConfig().slave_latency, timeout);
      }
      
      void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override { 
            ble_instance->setConnected(false); 
            NimBLEDevice::startAdvertising(); 
      }
};

SixthSense_BLE::SixthSense_BLE() {}

void SixthSense_BLE::begin(const BLE_Config_t &cfg) {
      ble_config = cfg;
      
      NimBLEDevice::init(config.device_name.c_str());
      NimBLEDevice::setPower(config.tx_power_level); 

      pServer = NimBLEDevice::createServer();
      pServer->setCallbacks(new ServerCallbacks(this));
      
      NimBLEService *pService = pServer->createService(BLE_SERVICE_UUID);
      pTxCharacteristic = pService->createCharacteristic(
                                      BLE_CHARACTERISTIC_UUID_TX,
                                      NIMBLE_PROPERTY::NOTIFY
                                  );
      
      pService->start();
      
      NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
      pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
      
      // Convert ms to 0.625ms advertising units
      uint16_t adv_int = config.adv_interval_ms / 0.625;
      pAdvertising->setMinInterval(adv_int);
      pAdvertising->setMaxInterval(adv_int);
      
      pAdvertising->start();
      Serial.println("[BLE] Bluetooth device initialized and advertising.");
}

void SixthSense_BLE::sendPayload(const std::string &payload) {
      if (connected && pTxCharacteristic != NULL && payload.length() > 0) {
            pTxCharacteristic->setValue(payload);
            pTxCharacteristic->notify();
      }
}