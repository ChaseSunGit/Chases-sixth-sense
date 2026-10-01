#include "BLE_Comm.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
      void onConnect(BLEServer* pServer) { deviceConnected = true; }
      void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

void BLE_init() {
      BLEDevice::init("ESP32_IMU");
      BLEServer *pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      BLEService *pService = pServer->createService(SERVICE_UUID);
      pCharacteristic = pService->createCharacteristic(
                              CHARACTERISTIC_UUID,
                              BLECharacteristic::PROPERTY_NOTIFY
                        );
      pCharacteristic->addDescriptor(new BLE2902());
      pService->start();
      pServer->getAdvertising()->start();
}

void BLE_toggle(bool state) {
      ble_enabled = state;
}

void BLE_send_sensor_data(float* quat, float roll, float pitch, float yaw) {
      if (deviceConnected && ble_enabled) {
            char txString[64];
            snprintf(txString, sizeof(txString), "R:%.2f P:%.2f Y:%.2f", roll, pitch, yaw);
            pCharacteristic->setValue(txString);
            pCharacteristic->notify();
      }
}