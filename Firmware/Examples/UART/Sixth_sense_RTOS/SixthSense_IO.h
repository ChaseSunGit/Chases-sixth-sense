/**
 * \file SixthSense_IO.h
 * \brief Communication handler for UART, CAN 2.0 (TWAI), and Wireless (ESPNOW).
 *
 * \author    Chase Sun
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
#include "SixthSense_Wireless.h"

#define CAN_TWAI_RX       GPIO_NUM_5
#define CAN_TWAI_TX       GPIO_NUM_6
#define PIN_WIRELESS_SET  GPIO_NUM_34
#define PIN_STATUS_LED    GPIO_NUM_2

#define MAX_PERIPHERAL_NODES 9
#define TOTAL_NETWORK_NODES  (MAX_PERIPHERAL_NODES + 1)

enum NodeMode : uint8_t {
      NODE_CENTRAL = 0,
      NODE_PERIPHERAL = 1,
}; 

struct IO_Config_t {
      uint16_t local_can_id;                              // 11-bit Base ID (multiple of 10, max 2040)
      uint16_t peripheral_can_ids[MAX_PERIPHERAL_NODES];  // 11-bit Base IDs for up to 9 peripherals
      int peripheral_timeout;                             // Milliseconds elapsed after valid measurement to mark peripheral inactive, 0 means no timeout
      uint8_t num_peripherals;                            // Number of configured peripherals (default 5, max 9)
      NodeMode mode;                                      // 0: Central, 1: Peripheral
      bool CAN_1MHZ_en;                                   // Enable 1MHz CAN speed, default enable
      bool write_accelGyro;
      bool write_mag;
      bool write_euler;
      bool write_quat;
      bool verbose_uart;
      bool listener_mode;
};

class SixthSense_IO {
public:
      SixthSense_IO();

      // Initialize IO (checks IO34 to latch Wired TWAI vs Wireless ESP-NOW)
      bool begin(const IO_Config_t &config, const Wireless_Config_t &wireless_config);

      // Provide mailbox handle to the IMU driver
      QueueHandle_t getIMUDataQueue() const { return imu_data_queue; }

      // Task lifecycle
      bool RTOS_startIOTasks(BaseType_t core_id = 0, UBaseType_t priority = 4);
      void RTOS_stopIOTasks();

      // Pause/Resume data transmission (LED stays solid ON when paused)
      void setDataTransferPaused(bool paused) { tx_paused = paused; }
      bool isWirelessMode() const { return wireless_mode_enabled; }

      static void returnDefaultConfig(IO_Config_t &io_config);

private:
      IO_Config_t io_config;
      SixthSense_Wireless wireless;
      bool wireless_mode_enabled = false;
      volatile bool tx_paused = false; // used for tracking led heartbeat task
      volatile uint32_t last_tx_activity_ms = 0;

      QueueHandle_t imu_data_queue = NULL;

      // Index 0: Local Node; Indices 1 to 9: Peripheral Nodes
      Fusion_Data_t network_imu_data[TOTAL_NETWORK_NODES];
      uint16_t network_soc_data[TOTAL_NETWORK_NODES];
      bool network_active_channel[TOTAL_NETWORK_NODES];

      TaskHandle_t tx_task_handle = NULL;
      TaskHandle_t rx_task_handle = NULL;
      TaskHandle_t led_task_handle = NULL;

      static void TX_Task_Trampoline(void *pvParameters);
      static void RX_Task_Trampoline(void *pvParameters);
      static void LED_Task_Trampoline(void *pvParameters);

      void transmitTask();
      void receiveTask();
      void ledBlinkTask();

      inline void sendCANMessage(uint16_t msg_id, float f1, float f2);
      void printUARTData();
      void printUARTData_Debug();

      uint16_t assembleBinaryData(uint8_t* target_buf, size_t max_len);
      void printBinaryData();
      int getPeripheralIndex(uint16_t base_id);

      bool disassembleWirelessData(const uint8_t* src_buf, uint16_t len, const uint8_t* sender_mac = nullptr);

      uint32_t lastTime_millis = 0;
      uint16_t adc_sample_counter = 0;
};

#endif // SIXTHSENSE_IO_H