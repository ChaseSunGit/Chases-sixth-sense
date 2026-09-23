#include "ImuDmaDriver.h"

ImuDmaDriver::ImuDmaDriver(const Config &cfg)
      : _cfg(cfg),
            _total_dma_len(cfg.burst_data_len + 1),
            _spi_handle(NULL),
            _tx_dma_buf(NULL),
            _rx_dma_buf(NULL),
            _hTask(NULL) {}

ImuDmaDriver::~ImuDmaDriver() {
      if (_hTask) vTaskDelete(_hTask);
      if (_spi_handle) spi_bus_remove_device(_spi_handle);
      if (_tx_dma_buf) heap_caps_free(_tx_dma_buf);
      if (_rx_dma_buf) heap_caps_free(_rx_dma_buf);
}

esp_err_t ImuDmaDriver::begin() {
      esp_err_t ret;

      // 1. Allocate DMA Buffers in internal SRAM
      _tx_dma_buf = (uint8_t *)heap_caps_malloc(_total_dma_len, MALLOC_CAP_DMA);
      _rx_dma_buf = (uint8_t *)heap_caps_malloc(_total_dma_len, MALLOC_CAP_DMA);
      if (!_tx_dma_buf || !_rx_dma_buf) return ESP_ERR_NO_MEM;

      // Pre-load TX buffer with Read Command + 0x00 clocks
      memset(_tx_dma_buf, 0x00, _total_dma_len);
      _tx_dma_buf[0] = _cfg.burst_start_reg | 0x80;

      // 2. Pre-configure the DMA Transaction descriptor
      memset(&_dma_trans, 0, sizeof(spi_transaction_t));
      _dma_trans.length = _total_dma_len * 8; // Bits
      _dma_trans.tx_buffer = _tx_dma_buf;
      _dma_trans.rx_buffer = _rx_dma_buf;
      _dma_trans.user = (void *)this; // Pass instance pointer into transaction context

      // 3. Initialize the SPI Bus
      spi_bus_config_t buscfg = {};
      buscfg.mosi_io_num = _cfg.pin_mosi;
      buscfg.miso_io_num = _cfg.pin_miso;
      buscfg.sclk_io_num = _cfg.pin_sclk;
      buscfg.quadwp_io_num = -1;
      buscfg.quadhd_io_num = -1;
      buscfg.max_transfer_sz = _total_dma_len;

      ret = spi_bus_initialize(_cfg.host, &buscfg, SPI_DMA_CH_AUTO);
      if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret; // ESP_ERR_INVALID_STATE if bus already initialized

      // 4. Attach Device to SPI Bus with post_cb hook
      spi_device_interface_config_t devcfg = {};
      devcfg.clock_speed_hz = _cfg.spi_clock_hz;
      devcfg.mode = 0; // SPI Mode 0
      devcfg.spics_io_num = _cfg.pin_cs;
      devcfg.queue_size = 4;
      devcfg.post_cb = ImuDmaDriver::spi_post_cb;

      ret = spi_bus_add_device(_cfg.host, &devcfg, &_spi_handle);
      if (ret != ESP_OK) return ret;

      // 5. Create Worker Task
      xTaskCreatePinnedToCore(
            ImuDmaDriver::task_entry,
            "IMU_Task",
            4096,
            this,
            _cfg.task_priority,
            &_hTask,
            _cfg.core_id
      );

      // 6. Setup DRDY GPIO & ISR
      gpio_config_t io_conf = {};
      io_conf.intr_type = GPIO_INTR_POSEDGE;
      io_conf.pin_bit_mask = (1ULL << _cfg.pin_drdy);
      io_conf.mode = GPIO_MODE_INPUT;
      io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
      gpio_config(&io_conf);

      gpio_install_isr_service(0);
      gpio_isr_handler_add((gpio_num_t)_cfg.pin_drdy, ImuDmaDriver::gpio_isr_handler, (void *)this);

      return ESP_OK;
}

// --------------------------------------------------------------------------
// Synchronous Helpers
// --------------------------------------------------------------------------
esp_err_t ImuDmaDriver::writeRegister(uint8_t reg, uint8_t val) {
      uint8_t *tx = (uint8_t *)heap_caps_malloc(2, MALLOC_CAP_DMA);
      tx[0] = reg & 0x7F; // Write flag
      tx[1] = val;

      spi_transaction_t trans = {};
      trans.length = 16;
      trans.tx_buffer = tx;

      esp_err_t ret = spi_device_polling_transmit(_spi_handle, &trans);
      heap_caps_free(tx);
      return ret;
}

esp_err_t ImuDmaDriver::readRegister(uint8_t reg, uint8_t *val) {
      uint8_t *tx = (uint8_t *)heap_caps_malloc(2, MALLOC_CAP_DMA);
      uint8_t *rx = (uint8_t *)heap_caps_malloc(2, MALLOC_CAP_DMA);
      tx[0] = reg | 0x80; // Read flag
      tx[1] = 0x00;

      spi_transaction_t trans = {};
      trans.length = 16;
      trans.tx_buffer = tx;
      trans.rx_buffer = rx;

      esp_err_t ret = spi_device_polling_transmit(_spi_handle, &trans);
      if (ret == ESP_OK) {
            *val = rx[1]; // Byte 0 is garbage
      }

      heap_caps_free(tx);
      heap_caps_free(rx);
      return ret;
}

// --------------------------------------------------------------------------
// Static Trampolines & Callbacks
// --------------------------------------------------------------------------

// Hardware DRDY Interrupt
void IRAM_ATTR ImuDmaDriver::gpio_isr_handler(void *arg) {
      ImuDmaDriver *instance = reinterpret_cast<ImuDmaDriver *>(arg);

      // Queue non-blocking DMA read from ISR context
      spi_device_queue_trans(instance->_spi_handle, &instance->_dma_trans, 0);
}

// Hardware SPI DMA Completion Interrupt
void IRAM_ATTR ImuDmaDriver::spi_post_cb(spi_transaction_t *trans) {
      ImuDmaDriver *instance = reinterpret_cast<ImuDmaDriver *>(trans->user);
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;

      // Notify the processing task
      vTaskNotifyGiveFromISR(instance->_hTask, &xHigherPriorityTaskWoken);

      if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
      }
}

// Processing Task Loop
void ImuDmaDriver::task_entry(void *pvParameters) {
      ImuDmaDriver *instance = reinterpret_cast<ImuDmaDriver *>(pvParameters);

      for (;;) {
            // Sleep until DMA completes
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Retrieve SPI descriptor to flush completed status
            spi_transaction_t *completed_trans;
            spi_device_get_trans_result(instance->_spi_handle, &completed_trans, 0);

            // Unpack big-endian register pairs (skipping index 0 dummy byte)
            uint8_t *raw = &instance->_rx_dma_buf[1];
                
            ImuRawFrame frame;
            frame.timestamp_us = micros();
            frame.accel_x = (int16_t)((raw[0]        << 8) | raw[1]);
            frame.accel_y = (int16_t)((raw[2]        << 8) | raw[3]);
            frame.accel_z = (int16_t)((raw[4]        << 8) | raw[5]);
            frame.temp                = (int16_t)((raw[6]        << 8) | raw[7]);
            frame.gyro_x        = (int16_t)((raw[8]        << 8) | raw[9]);
            frame.gyro_y        = (int16_t)((raw[10] << 8) | raw[11]);
            frame.gyro_z        = (int16_t)((raw[12] << 8) | raw[13]);

            instance->onDataReady(frame);
      }
}

void ImuDmaDriver::onDataReady(const ImuRawFrame &frame) {
      // Default base implementation: empty (or simple debugging)
}