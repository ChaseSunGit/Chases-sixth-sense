#include "CAN_Comm.h"
#include "driver/twai.h"
#include <string.h>

#define TX_PIN GPIO_NUM_5
#define RX_PIN GPIO_NUM_4

void CAN_init() {
      twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
      twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
      twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

      if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
            twai_start();
      }
}

void CAN_send_sensor_data(float* quat, float roll, float pitch, float yaw) {
      twai_message_t message;
      message.identifier = 0x100; // Example ID
      message.extd = 0;
      message.data_length_code = 8;
      
      // Example: Packing roll and pitch into 8 bytes
      int32_t r_int = (int32_t)(roll * 100);
      int32_t p_int = (int32_t)(pitch * 100);
      
      memcpy(&message.data[0], &r_int, 4);
      memcpy(&message.data[4], &p_int, 4);

      twai_transmit(&message, pdMS_TO_TICKS(0));
}