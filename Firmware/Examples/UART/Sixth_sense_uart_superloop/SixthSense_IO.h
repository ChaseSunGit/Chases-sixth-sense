/**
 * \file SixthSense_IO.h
 * \brief Communication handler for UART, CAN 2.0 (TWAI), and future BLE.
 */

#ifndef SIXTHSENSE_IO_H
#define SIXTHSENSE_IO_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <cstdint>
#include "driver/twai.h"
#include "SixthSense_IMU.h"

#define CAN_TWAI_RX GPIO_NUM_5
#define CAN_TWAI_TX GPIO_NUM_6

enum NodeMode : uint8_t {
      NODE_CENTRAL = 0,
      NODE_PERIPHERAL = 1,
      NODE_BLE = 2
}; 

struct IO_Config_t {
      uint16_t local_can_id;          // 11-bit Base ID (multiple of 10, max 2040)
      uint16_t peripheral_can_ids[5]; // 11-bit Base IDs for up to 5 peripherals 
      NodeMode mode;                  // 8-bit int to select mode of IO
      bool CAN_1MHZ_en;               // enable 1Mhz CAN speed, default disable - 500KHz
      bool write_accelGyro;
      bool write_mag;
      bool write_euler;
      bool write_quat;
      bool verbose_uart;
};

class SixthSense_IO {
public:
      SixthSense_IO();

      // Initialize TWAI and create the internal 1-slot mailbox queue
      bool begin(const IO_Config_t &config);

      // Provide mailbox handle to the IMU driver
      QueueHandle_t getIMUDataQueue() const { return imu_data_queue; }

      // Task lifecycle
      bool RTOS_startIOTasks(BaseType_t core_id = 0, UBaseType_t priority = 4);
      void RTOS_stopIOTasks();

      static void returnDefaultConfig(IO_Config_t &io_config);


private:
      IO_Config_t io_config;
      QueueHandle_t imu_data_queue = NULL;

      // Index 0: Local Node; Indices 1 to 5: Peripheral Nodes
      Fusion_Data_t can_network_imu_data[6];
      bool active_can_channel[6];

      TaskHandle_t tx_task_handle = NULL;
      TaskHandle_t rx_task_handle = NULL;

      static void TX_Task_Trampoline(void *pvParameters);
      static void RX_Task_Trampoline(void *pvParameters);

      void transmitTask();
      void receiveTask();

      inline void sendCANMessage(uint16_t msg_id, float f1, float f2);
      void printUARTData();
      void printUARTData_Debug();
      int getPeripheralIndex(uint16_t base_id);

      uint32_t lastTime_millis;
};

#endif // SIXTHSENSE_IO_H