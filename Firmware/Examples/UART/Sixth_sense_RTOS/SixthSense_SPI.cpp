/**
 * \file      SixthSense_SPI.cpp
 * \brief     Library for SPI interactions
 *
 * \author    Chase Sun
 */

#include "SixthSense_SPI.h"


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
      buscfg.quadwp_io_num = -1; 
      buscfg.quadhd_io_num = -1;
      buscfg.max_transfer_sz = max_transfer_sz;

      // Disable DMA for standard polling transactions
      esp_err_t err = spi_bus_initialize(host, &buscfg, SPI_DMA_DISABLED);

      if (err == ESP_ERR_INVALID_STATE) {
            Serial.printf("[SPI Bus] Host %d is already initialized. Skipping re-initialization.\n", host);
            return true; 
      }
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
 * \param out_handle        Reference to a spi_device_handle_t variable where the initialized driver handle will be stored.
 * \param clock_speed_hz    SPI clock frequency in Hertz. Defaults to 10000000 (10 MHz).
 * \param mode              SPI operating mode (0, 1, 2, or 3) setting CPOL and CPHA. Defaults to 0.
 * \param queue_size        Depth of the transaction queue (how many transactions can be pending/in-flight). Defaults to 3.
 * \param host              Hardware SPI host peripheral where the device is physically connected. Defaults to SPI2_HOST.
 * \return true if device was registered and attached successfully, false otherwise.
 */
bool SPI_Add_Device(int cs_pin,
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
      // post_cb removed as DMA completion callbacks are no longer used

      esp_err_t err = spi_bus_add_device(host, &devcfg, &out_handle);
      if (err != ESP_OK) {
            Serial.printf("[SPI Dev] Failed to add device on CS %d: 0x%X\n", cs_pin, err);
            return false;
      }
      return true;
}

