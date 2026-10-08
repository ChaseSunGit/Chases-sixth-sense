/**
 * \file      Sixth_sense_SPI.cpp
 * \brief     Library for SPI interactions
 *
 * \author    Chase Sun
 */

#include "Sixth_sense_SPI.h"


/**
      Explanation of SPI setup process:

      The SPI initialization process on the ESP32S3 requires 3 steps: Initializing the bus, initializing the device, and initializing the DMA channel.

      1.    Initializing the bus: The ESP32S3 has 2 SPI hosts: SPI2_HOST and SPI3_HOST, and each one can run a group of devices with shared data and clock lines.
      2.    Initializing the device: Each device (tied to CC lines) are defined using this function including speed and call back function when transaction completes
      3.    Initializing the DMA channel: arms a specific DMA channel for a specific transaction by pre-allocating both rx and tx buffers from heap memory and 
            populating the tx buffer with the read command and a bunch of 0s. The transaction is then returned and can be called for requesting DMA reads.
**/

/**
 * \brief Initializes a hardware SPI host bus with automatic DMA channel allocation.
 *
 * \param mosi_pin          GPIO pin number assigned to SPI Master Out Slave In (MOSI).
 * \param miso_pin          GPIO pin number assigned to SPI Master In Slave Out (MISO).
 * \param sclk_pin          GPIO pin number assigned to SPI Serial Clock (SCLK).
 * \param max_transfer_sz   Maximum transfer size in bytes allowed on this bus. Defaults to 4096.
 * \param host              Hardware SPI host peripheral to use (e.g., SPI2_HOST or SPI3_HOST). Defaults to SPI2_HOST.
 * \return true if the bus was initialized successfully or was already initialized; false on configuration failure.
 */
bool SPI_Bus_Init(int mosi_pin, 
                  int miso_pin, 
                  int sclk_pin, 
                  int max_transfer_sz,
                  spi_host_device_t host) {

      spi_bus_config_t buscfg = {};
      buscfg.mosi_io_num = mosi_pin;
      buscfg.miso_io_num = miso_pin;
      buscfg.sclk_io_num = sclk_pin;
      buscfg.quadwp_io_num = -1; //These arguments turn off quad-SPI mode and enables standard SPI
      buscfg.quadhd_io_num = -1;
      buscfg.max_transfer_sz = max_transfer_sz;

      esp_err_t err = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);

      // Check if host is initialized
      if (err == ESP_ERR_INVALID_STATE) {
            Serial.printf("[SPI Bus] Host %d is already initialized. Skipping re-initialization.\n", host);
            return true; 
      }

      // Check if initialization is valid
      if (err != ESP_OK) {
            Serial.printf("[SPI Bus] Init failed on host %d: 0x%X\n", host, err);
            return false;
      }
      return true;
}

/**
 * \brief Attaches and registers a specific slave device to an initialized SPI host bus.
 *
 * \param cs_pin            GPIO pin number assigned to Chip Select (CS / SS) for this specific slave device.
 * \param post_callback     ISR callback function (of type transaction_cb_t) invoked immediately when a DMA transfer completes.
 * \param out_handle        Reference to a spi_device_handle_t variable where the initialized driver handle will be stored.
 * \param clock_speed_hz    SPI clock frequency in Hertz. Defaults to 10000000 (10 MHz).
 * \param mode              SPI operating mode (0, 1, 2, or 3) setting CPOL and CPHA. Defaults to 0.
 * \param queue_size        Depth of the transaction queue (how many transactions can be pending/in-flight). Defaults to 3.
 * \param host              Hardware SPI host peripheral where the device is physically connected. Defaults to SPI2_HOST.
 * \return true if device was registered and attached successfully, false otherwise.
 */
bool SPI_Add_Device(int cs_pin,
                    transaction_cb_t post_callback,
                    spi_device_handle_t &out_handle,
                    int clock_speed_hz,
                    int mode,
                    int queue_size,
                    spi_host_device_t host) {

      spi_device_interface_config_t devcfg = {};
      devcfg.clock_speed_hz = clock_speed_hz;
      devcfg.mode = mode;
      devcfg.spics_io_num = cs_pin;
      devcfg.queue_size = queue_size;
      devcfg.post_cb = post_callback;

      esp_err_t err = spi_bus_add_device(host, &devcfg, &out_handle);
      if (err != ESP_OK) {
            Serial.printf("[SPI Dev] Failed to add device on CS %d: 0x%X\n", cs_pin, err);
            return false;
      }
      return true;
}

/**
 * \brief Allocates DMA-capable internal SRAM, pre-arms the transmit payload, and sets up a persistent transaction descriptor.
 *
 * \param burst_len         Total transaction length in bytes (including command/address header and data bytes).
 * \param initial_cmd_byte  Leading register address byte (and any read/write flags) written to tx_buffer[0].
 * \param out_channel       Reference to an SPI_DMA_Channel container populated with allocated buffers and descriptors.
 * \return true if internal DMA memory was allocated and descriptors configured successfully, false if heap allocation failed.
 */
bool SPI_Arm_DMA_Channel(size_t burst_len,
                         uint8_t initial_cmd_byte,
                         SPI_DMA_Channel &out_channel) {

      if (out_channel.handle == NULL) {
            Serial.println("[SPI DMA] Error: Device handle is NULL. Call SPI_Add_Device first!");
            return false;
      }

      out_channel.buffer_len = burst_len;
      out_channel.in_progress = false;
      out_channel.data_ready = false;

      // Allocate memory from internal SRAM satisfying hardware DMA alignment constraints
      out_channel.tx_buffer = (uint8_t *)heap_caps_malloc(burst_len, MALLOC_CAP_DMA);
      out_channel.rx_buffer = (uint8_t *)heap_caps_malloc(burst_len, MALLOC_CAP_DMA);

      if (!out_channel.tx_buffer || !out_channel.rx_buffer) {
            Serial.println("[SPI DMA] Failed to allocate DMA-accessible internal memory.");
            SPI_Free_DMA_Channel(out_channel);
            return false;
      }

      // Zero-initialize buffers and set initial command/address byte
      memset(out_channel.tx_buffer, 0, burst_len);
      out_channel.tx_buffer[0] = initial_cmd_byte;
      memset(out_channel.rx_buffer, 0, burst_len);

      // Pre-arm the reusable SPI transaction descriptor
      memset(&out_channel.trans, 0, sizeof(spi_transaction_t));
      out_channel.trans.length = burst_len * 8; // SPI master driver takes transaction bit length
      out_channel.trans.tx_buffer = out_channel.tx_buffer;
      out_channel.trans.rx_buffer = out_channel.rx_buffer;
      out_channel.trans.user = (void *)&out_channel; // Pointer back to this container for retrieval inside callbacks

      return true;
}

/**
 * \brief Safely releases heap-allocated DMA buffers and resets channel descriptors.
 *
 * \param channel Reference to the SPI_DMA_Channel object whose allocated memory will be freed.
 */
void SPI_Free_DMA_Channel(SPI_DMA_Channel &channel) {
      if (channel.tx_buffer != nullptr) {
            heap_caps_free(channel.tx_buffer);
            channel.tx_buffer = nullptr;
      }
      if (channel.rx_buffer != nullptr) {
            heap_caps_free(channel.rx_buffer);
            channel.rx_buffer = nullptr;
      }
      channel.buffer_len = 0;
      channel.in_progress = false;
      channel.data_ready = false;
}