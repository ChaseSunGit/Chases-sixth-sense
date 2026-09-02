#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "SPI_DMA_APP";

// --- Pin Definitions (Custom S3 Pinout) ---
#define PIN_SPI_MISO    GPIO_NUM_13
#define PIN_SPI_MOSI    GPIO_NUM_11
#define PIN_SPI_SCLK    GPIO_NUM_12
#define PIN_SPI_CS      GPIO_NUM_10

#define SPI_HOST_ID     SPI2_HOST
#define BUFFER_SIZE     16   // Bytes per SPI transaction

// --- Data Types ---
typedef struct {
    uint8_t payload[BUFFER_SIZE];
    int64_t timestamp_us;
} raw_sensor_packet_t;

typedef struct {
    float channel_a;
    float channel_b;
    float computed_magnitude;
    int64_t timestamp_us;
} processed_data_t;

// --- FreeRTOS Handles & Buffers ---
static QueueHandle_t s_sensor_queue = NULL;
static spi_device_handle_t s_spi_dev = NULL;

// DMA buffers in internal 32-bit aligned SRAM
static uint8_t *s_tx_dma_buf = NULL;
static uint8_t *s_rx_dma_buf = NULL;

// --- Calculation Function ---
static inline processed_data_t process_sensor_stream(const raw_sensor_packet_t *raw) {
    processed_data_t result;
    result.timestamp_us = raw->timestamp_us;

    // Unpack big-endian 16-bit signed registers (e.g., raw sensor data)
    int16_t raw_val_1 = (int16_t)((raw->payload[1] << 8) | raw->payload[2]);
    int16_t raw_val_2 = (int16_t)((raw->payload[3] << 8) | raw->payload[4]);

    // Apply scale factor (e.g., ±16g scale)
    const float scale_factor = 0.001952f;
    result.channel_a = (float)raw_val_1 * scale_factor;
    result.channel_b = (float)raw_val_2 * scale_factor;

    // Vector magnitude calculation
    result.computed_magnitude = sqrtf((result.channel_a * result.channel_a) + 
                                      (result.channel_b * result.channel_b));

    return result;
}

// --- SPI DMA Hardware Initialization ---
static esp_err_t init_spi_dma(void) {
    // 1. Allocate DMA-capable memory in internal SRAM
    s_tx_dma_buf = (uint8_t *)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
    s_rx_dma_buf = (uint8_t *)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);

    if (!s_tx_dma_buf || !s_rx_dma_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers!");
        return ESP_ERR_NO_MEM;
    }

    // 2. Configure SPI Bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE,
    };

    // Enable SPI DMA auto channel selection
    esp_err_t ret = spi_bus_initialize(SPI_HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 3. Attach Sensor Device Interface
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 0,                          // SPI Mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = PIN_SPI_CS,
        .queue_size = 1,
    };

    ret = spi_bus_add_device(SPI_HOST_ID, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device registration failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// --- RTOS Tasks ---

// Task 1: Dedicated Hardware Acquisition (Core 0)
static void task_sensor_acquire(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t sample_period = pdMS_TO_TICKS(20); // 50 Hz loop

    spi_transaction_t trans = {
        .length = BUFFER_SIZE * 8, // Length in bits
        .tx_buffer = s_tx_dma_buf,
        .rx_buffer = s_rx_dma_buf,
    };

    while (1) {
        memset(s_tx_dma_buf, 0x00, BUFFER_SIZE);
        s_tx_dma_buf[0] = 0x80; // Sensor burst-read register command

        // Initiates DMA hardware transfer and yields task until complete
        esp_err_t ret = spi_device_transmit(s_spi_dev, &trans);

        if (ret == ESP_OK) {
            raw_sensor_packet_t packet;
            packet.timestamp_us = esp_timer_get_time();
            memcpy(packet.payload, s_rx_dma_buf, BUFFER_SIZE);

            // Push to processing task; discard oldest if queue is full
            xQueueSend(s_sensor_queue, &packet, 0);
        } else {
            ESP_LOGW(TAG, "SPI transmit error: %s", esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last_wake_time, sample_period);
    }
}

// Task 2: Math Processing & USB Stream Output (Core 1)
static void task_data_process(void *pvParameters) {
    raw_sensor_packet_t raw_packet;

    while (1) {
        if (xQueueReceive(s_sensor_queue, &raw_packet, portMAX_DELAY) == pdTRUE) {
            processed_data_t processed = process_sensor_stream(&raw_packet);

            // Print directly to USB Serial terminal
            printf("[%8lld us] ChA: %+0.3f | ChB: %+0.3f | Mag: %0.3f\n",
                   processed.timestamp_us,
                   processed.channel_a,
                   processed.channel_b,
                   processed.computed_magnitude);
        }
    }
}

// --- Application Entry Point ---
void app_main(void) {
    ESP_LOGI(TAG, "Booting ESP-IDF SPI DMA pipeline...");

    // Create FreeRTOS queue (capacity: 10 items)
    s_sensor_queue = xQueueCreate(10, sizeof(raw_sensor_packet_t));
    if (!s_sensor_queue) {
        ESP_LOGE(TAG, "Queue allocation failed!");
        return;
    }

    // Initialize SPI bus with DMA
    ESP_ERROR_CHECK(init_spi_dma());

    // Task Pinning: Core 0 (I/O & DMA triggers), Core 1 (Floating point math & printing)
    xTaskCreatePinnedToCore(task_sensor_acquire, "spi_acq_task",  4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(task_data_process,   "data_proc_task", 4096, NULL, 2, NULL, 1);
}