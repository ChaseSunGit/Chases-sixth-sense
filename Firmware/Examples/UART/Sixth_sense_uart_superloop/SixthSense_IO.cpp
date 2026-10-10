/**
 * \file SixthSense_IO.cpp
 * \brief Implementation of the communication and packing tasks.
 */

#include "SixthSense_IO.h"
#include <string.h>

SixthSense_IO::SixthSense_IO() {
    memset(network_data, 0, sizeof(network_data));
    memset(data_active, 0, sizeof(data_active));
}

bool SixthSense_IO::begin(const IO_Config_t &config) {
    io_config = config;
    data_active[0] = true; // Local data is always active

    // Initialize TWAI (CAN 2.0) driver for 1 Mbps 
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(io_config.can_tx_pin, io_config.can_rx_pin, TWAI_MODE_NORMAL);
    
    // A Central node needs a larger RX queue to handle burst traffic from 5 peripherals
    g_config.rx_queue_len = (io_config.mode == NODE_CENTRAL) ? 50 : 10;
    g_config.tx_queue_len = 10;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // Adjust to your bus speed
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[IO] Failed to install TWAI driver");
        return false;
    }
    
    if (twai_start() != ESP_OK) {
        Serial.println("[IO] Failed to start TWAI driver");
        return false;
    }

    Serial.println("[IO] CAN/TWAI interface initialized successfully.");
    return true;
}

void SixthSense_IO::triggerTransmission(const Fusion_Data_t &local_data) {
    // Safely cache local data for the TX task
    network_data[0] = local_data;

    if (tx_task_handle != NULL) {
        xTaskNotifyGive(tx_task_handle);
    }
}

bool SixthSense_IO::RTOS_startIOTasks(BaseType_t core_id, UBaseType_t priority) {
    if (tx_task_handle != NULL || rx_task_handle != NULL) return true;

    BaseType_t tx_res = xTaskCreatePinnedToCore(
        TX_Task_Trampoline, "IO_TX_Task", 4096, this, priority, &tx_task_handle, core_id
    );

    // RX task gets a slightly higher priority to ensure incoming bus buffers don't overflow
    BaseType_t rx_res = xTaskCreatePinnedToCore(
        RX_Task_Trampoline, "IO_RX_Task", 4096, this, priority + 1, &rx_task_handle, core_id
    );

    return (tx_res == pdPASS && rx_res == pdPASS);
}

void SixthSense_IO::TX_Task_Trampoline(void *pvParameters) {
    SixthSense_IO *io = static_cast<SixthSense_IO*>(pvParameters);
    io->transmitTask();
}

void SixthSense_IO::RX_Task_Trampoline(void *pvParameters) {
    SixthSense_IO *io = static_cast<SixthSense_IO*>(pvParameters);
    io->receiveTask();
}

void SixthSense_IO::transmitTask() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Fusion_Data_t &loc = network_data[0];
        uint32_t base = io_config.local_can_id;

        // 1. Pack and push configured CAN frames to the bus
        if (io_config.can_message_mask & CAN_MSG_X1_ACCELX_GYROX) sendCANMessage(base + 1, loc.accel[0], loc.gyro[0]);
        if (io_config.can_message_mask & CAN_MSG_X2_ACCELY_GYROY) sendCANMessage(base + 2, loc.accel[1], loc.gyro[1]);
        if (io_config.can_message_mask & CAN_MSG_X3_ACCELZ_GYROZ) sendCANMessage(base + 3, loc.accel[2], loc.gyro[2]);
        if (io_config.can_message_mask & CAN_MSG_X4_MAGX_MAGY)    sendCANMessage(base + 4, loc.mag[0], loc.mag[1]);
        if (io_config.can_message_mask & CAN_MSG_X5_MAGZ)         sendCANMessage(base + 5, loc.mag[2], 0.0f);
        if (io_config.can_message_mask & CAN_MSG_X6_ROLL_PITCH)   sendCANMessage(base + 6, loc.roll, loc.pitch);
        if (io_config.can_message_mask & CAN_MSG_X7_YAW)          sendCANMessage(base + 7, loc.yaw, 0.0f);
        if (io_config.can_message_mask & CAN_MSG_X8_QUAT0_QUAT1)  sendCANMessage(base + 8, loc.q[0], loc.q[1]);
        if (io_config.can_message_mask & CAN_MSG_X9_QUAT2_QUAT3)  sendCANMessage(base + 9, loc.q[2], loc.q[3]);

        // 2. If acting as Central Node, flush all known network data to UART
        if (io_config.mode == NODE_CENTRAL) {
            printUARTData();
        }
    }
}

void SixthSense_IO::receiveTask() {
    twai_message_t rx_msg;
    
    while (1) {
        // Block until a CAN message arrives
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            if (rx_msg.data_length_code != 8) continue; // Expecting exactly 2 floats (8 bytes)

            // Calculate which peripheral this belongs to
            uint32_t base_id = rx_msg.identifier - (rx_msg.identifier % 10);
            uint8_t msg_type = rx_msg.identifier % 10;
            
            int p_idx = getPeripheralIndex(base_id);
            if (p_idx == -1) continue; // Unknown node, discard

            data_active[p_idx] = true;
            
            float f1, f2;
            memcpy(&f1, &rx_msg.data[0], sizeof(float));
            memcpy(&f2, &rx_msg.data[4], sizeof(float));

            // Map data into the structural registry based on message type
            switch(msg_type) {
                case 1: network_data[p_idx].accel[0] = f1; network_data[p_idx].gyro[0] = f2; break;
                case 2: network_data[p_idx].accel[1] = f1; network_data[p_idx].gyro[1] = f2; break;
                case 3: network_data[p_idx].accel[2] = f1; network_data[p_idx].gyro[2] = f2; break;
                case 4: network_data[p_idx].mag[0] = f1;   network_data[p_idx].mag[1] = f2;  break;
                case 5: network_data[p_idx].mag[2] = f1;                                     break;
                case 6: network_data[p_idx].roll = f1;     network_data[p_idx].pitch = f2;   break;
                case 7: network_data[p_idx].yaw = f1;                                        break;
                case 8: network_data[p_idx].q[0] = f1;     network_data[p_idx].q[1] = f2;    break;
                case 9: network_data[p_idx].q[2] = f1;     network_data[p_idx].q[3] = f2;    break;
            }
        }
    }
}

void SixthSense_IO::sendCANMessage(uint32_t msg_id, float f1, float f2) {
    twai_message_t tx_msg = {0};
    tx_msg.identifier = msg_id;
    tx_msg.data_length_code = 8;
    tx_msg.extd = 0; // Standard 11-bit ID

    memcpy(&tx_msg.data[0], &f1, sizeof(float));
    memcpy(&tx_msg.data[4], &f2, sizeof(float));

    // Non-blocking transmit, ticks to wait = 0
    twai_transmit(&tx_msg, 0);
}

int SixthSense_IO::getPeripheralIndex(uint32_t base_id) {
    for (int i = 0; i < 5; i++) {
        if (io_config.peripheral_can_ids[i] == base_id) return i + 1; // +1 offset for network_data array
    }
    return -1;
}

void SixthSense_IO::printUARTData() {
    // Prints a condensed CSV style stream. Each line starts with the Node Index (0=Local, 1-5=Peripherals)
    for (int i = 0; i < 6; i++) {
        if (!data_active[i]) continue;
        
        Fusion_Data_t &d = network_data[i];
        
        Serial.printf("NODE:%d,", i);
        
        if (io_config.can_message_mask & CAN_MSG_X1_ACCELX_GYROX) Serial.printf("%.4f,%.4f,", d.accel[0], d.gyro[0]); else Serial.print(",,");
        if (io_config.can_message_mask & CAN_MSG_X2_ACCELY_GYROY) Serial.printf("%.4f,%.4f,", d.accel[1], d.gyro[1]); else Serial.print(",,");
        if (io_config.can_message_mask & CAN_MSG_X3_ACCELZ_GYROZ) Serial.printf("%.4f,%.4f,", d.accel[2], d.gyro[2]); else Serial.print(",,");
        if (io_config.can_message_mask & CAN_MSG_X4_MAGX_MAGY)    Serial.printf("%.4f,%.4f,", d.mag[0], d.mag[1]); else Serial.print(",,");
        if (io_config.can_message_mask & CAN_MSG_X5_MAGZ)         Serial.printf("%.4f,", d.mag[2]); else Serial.print(",");
        if (io_config.can_message_mask & CAN_MSG_X6_ROLL_PITCH)   Serial.printf("%.4f,%.4f,", d.roll, d.pitch); else Serial.print(",,");
        if (io_config.can_message_mask & CAN_MSG_X7_YAW)          Serial.printf("%.4f,", d.yaw); else Serial.print(",");
        if (io_config.can_message_mask & CAN_MSG_X8_QUAT0_QUAT1)  Serial.printf("%.4f,%.4f,", d.q[0], d.q[1]); else Serial.print(",,");
        if (io_config.can_message_mask & CAN_MSG_X9_QUAT2_QUAT3)  Serial.printf("%.4f,%.4f", d.q[2], d.q[3]); else Serial.print(",");
        
        Serial.println();
    }
}