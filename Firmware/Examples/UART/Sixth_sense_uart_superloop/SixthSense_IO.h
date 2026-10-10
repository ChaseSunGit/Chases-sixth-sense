/**
 * \file SixthSense_IO.h
 * \brief Communication handler for UART, CAN 2.0 (TWAI), and future BLE.
 */

#ifndef SIXTHSENSE_IO_H
#define SIXTHSENSE_IO_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/twai.h"
#include "SixthSense_IMU.h"

// Define node operating modes
enum NodeMode {
    NODE_PERIPHERAL = 0,
    NODE_CENTRAL = 1
};

// Message bitmask definitions
#define CAN_MSG_X1_ACCELX_GYROX (1 << 0)
#define CAN_MSG_X2_ACCELY_GYROY (1 << 1)
#define CAN_MSG_X3_ACCELZ_GYROZ (1 << 2)
#define CAN_MSG_X4_MAGX_MAGY    (1 << 3)
#define CAN_MSG_X5_MAGZ         (1 << 4)
#define CAN_MSG_X6_ROLL_PITCH   (1 << 5)
#define CAN_MSG_X7_YAW          (1 << 6)
#define CAN_MSG_X8_QUAT0_QUAT1  (1 << 7)
#define CAN_MSG_X9_QUAT2_QUAT3  (1 << 8)

struct IO_Config_t {
    NodeMode mode;
    uint32_t local_can_id;          // Must be a multiple of 10
    uint32_t peripheral_can_ids[5]; // Up to 5 peripherals monitored by Central
    uint16_t can_message_mask;      // Upgraded to 16-bit to cover 9 messages
    gpio_num_t can_tx_pin;
    gpio_num_t can_rx_pin;
};

class SixthSense_IO {
public:
    SixthSense_IO();

    // Initialize the CAN driver and peripherals
    bool begin(const IO_Config_t &config);

    // RTOS Task Lifecycle
    bool RTOS_startIOTasks(BaseType_t core_id = 0, UBaseType_t priority = 4);
    void RTOS_stopIOTasks();

    // Triggered by SixthSense_IMU when a new data packet is ready
    void triggerTransmission(const Fusion_Data_t &local_data);

private:
    IO_Config_t io_config;
    
    // Data registry: Index 0 is local data, 1-5 are peripheral nodes
    Fusion_Data_t network_data[6]; 
    bool data_active[6]; // Tracks if a peripheral has reported in

    TaskHandle_t tx_task_handle = NULL;
    TaskHandle_t rx_task_handle = NULL;

    static void TX_Task_Trampoline(void *pvParameters);
    static void RX_Task_Trampoline(void *pvParameters);

    void transmitTask();
    void receiveTask();

    void sendCANMessage(uint32_t msg_id, float f1, float f2);
    void printUARTData();
    int getPeripheralIndex(uint32_t base_id);
};

#endif // SIXTHSENSE_IO_H