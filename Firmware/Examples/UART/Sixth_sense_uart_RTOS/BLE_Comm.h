#ifndef BLE_COMM_H
#define BLE_COMM_H

#include <Arduino.h>

extern bool ble_enabled;

void BLE_init();
void BLE_toggle(bool state);
void BLE_send_sensor_data(float* quat, float roll, float pitch, float yaw);

#endif