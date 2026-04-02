#pragma once

/**
 * @brief HAL de ESP-IDF para RadioLib
 *
 * Implementa la interfaz RadioLibHal usando los drivers nativos
 * de ESP-IDF (driver/gpio.h, driver/spi_master.h, esp_timer).
 *
 * El CS (NSS) lo maneja RadioLib manualmente vía GPIO,
 * NO a través del periférico SPI hardware.
 */

#include "RadioLib.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class EspHal : public RadioLibHal {
public:
    EspHal(int sck, int miso, int mosi)
        : RadioLibHal(GPIO_MODE_INPUT, GPIO_MODE_OUTPUT,
                      0, 1,
                      GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE),
          _sck(sck), _miso(miso), _mosi(mosi) {}

    void init() override { spiBegin(); }
    void term() override { spiEnd(); }

    // ── GPIO ────────────────────────────────────────────────────────────────

    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin == RADIOLIB_NC) return;
        gpio_reset_pin((gpio_num_t)pin);
        gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (pin == RADIOLIB_NC) return;
        gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override {
        if (pin == RADIOLIB_NC) return 0;
        return (uint32_t)gpio_get_level((gpio_num_t)pin);
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)mode);
        gpio_install_isr_service(0);
        gpio_isr_handler_add((gpio_num_t)interruptNum, (gpio_isr_t)interruptCb, nullptr);
    }

    void detachInterrupt(uint32_t interruptNum) override {
        gpio_isr_handler_remove((gpio_num_t)interruptNum);
    }

    // ── Tiempo ───────────────────────────────────────────────────────────────

    void delay(RadioLibTime_t ms) override {
        vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
    }

    void delayMicroseconds(RadioLibTime_t us) override {
        esp_rom_delay_us((uint32_t)us);
    }

    RadioLibTime_t millis() override {
        return (RadioLibTime_t)(esp_timer_get_time() / 1000ULL);
    }

    RadioLibTime_t micros() override {
        return (RadioLibTime_t)esp_timer_get_time();
    }

    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override {
        RadioLibTime_t s = micros();
        while (digitalRead(pin) == state)  if (micros() - s > timeout) return 0;
        while (digitalRead(pin) != state)  if (micros() - s > timeout) return 0;
        RadioLibTime_t p = micros();
        while (digitalRead(pin) == state)  if (micros() - s > timeout) return 0;
        return (long)(micros() - p);
    }

    // ── SPI ──────────────────────────────────────────────────────────────────
    // CS gestionado por RadioLib vía GPIO: spics_io_num = -1

    void spiBegin() override {
        if (_initialized) return;  // evitar doble init (RadioLib llama init() en cada begin())

        spi_bus_config_t bus = {};
        bus.miso_io_num   = _miso;
        bus.mosi_io_num   = _mosi;
        bus.sclk_io_num   = _sck;
        bus.quadwp_io_num = -1;
        bus.quadhd_io_num = -1;
        // Sin DMA: el driver ignora max_transfer_sz del usuario y lo fuerza a
        // SOC_SPI_MAXIMUM_BUFFER_SIZE (64 bytes = SPI_LL_CPU_MAX_BIT_LEN/8) que es el
        // límite real del FIFO de CPU. Las transferencias mayores se dividen en
        // spiTransfer(). Ponemos 64 para que el valor sea coherente con ese límite.
        bus.max_transfer_sz = 64;
        // SPI_DMA_DISABLED evita usar DMA, que requiere buffers alineados a 4 bytes
        // y RadioLib puede pasar punteros de pila sin garantía de alineación.
        esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE("EspHal", "spi_bus_initialize falló: %s", esp_err_to_name(ret));
            return;
        }

        spi_device_interface_config_t dev = {};
        dev.clock_speed_hz = 1000000;   // 1 MHz conservador
        dev.mode           = 0;
        dev.spics_io_num   = -1;        // CS gestionado por RadioLib vía GPIO
        dev.queue_size     = 1;
        dev.pre_cb         = nullptr;
        dev.post_cb        = nullptr;
        ret = spi_bus_add_device(SPI2_HOST, &dev, &_spi);
        if (ret != ESP_OK) {
            ESP_LOGE("EspHal", "spi_bus_add_device falló: %s", esp_err_to_name(ret));
            return;
        }

        _initialized = true;
    }

    void spiBeginTransaction() override {
        // Adquirir el bus SPI antes de la transacción (necesario con spics_io_num=-1)
        spi_device_acquire_bus(_spi, portMAX_DELAY);
    }

    void spiTransfer(uint8_t *out, size_t len, uint8_t *in) override {
        if (!_spi || len == 0) return;

        // El FIFO hardware en modo CPU (sin DMA) está limitado a 64 bytes
        // (SPI_LL_CPU_MAX_BIT_LEN = 512 bits). Dividimos aquí para que el
        // SX1276 reciba los bytes de forma continua mientras NSS permanece
        // en bajo (RadioLib no suelta NSS hasta spiEndTransaction).
        const size_t MAX_CHUNK = 64;
        size_t offset = 0;

        while (offset < len) {
            size_t chunk = (len - offset > MAX_CHUNK) ? MAX_CHUNK : (len - offset);

            spi_transaction_t t = {};
            t.length    = chunk * 8;
            t.tx_buffer = out ? (out + offset) : nullptr;
            t.rx_buffer = in  ? (in  + offset) : nullptr;

            // polling_transmit: sin DMA ni interrupciones, apto para cualquier contexto
            esp_err_t ret = spi_device_polling_transmit(_spi, &t);
            if (ret != ESP_OK) {
                ESP_LOGE("EspHal", "spiTransfer falló en offset %d: %s",
                         (int)offset, esp_err_to_name(ret));
                return;
            }
            offset += chunk;
        }
    }

    void spiEndTransaction() override {
        spi_device_release_bus(_spi);
    }

    void spiEnd() override {
        if (!_initialized) return;
        spi_bus_remove_device(_spi);
        spi_bus_free(SPI2_HOST);
        _spi = nullptr;
        _initialized = false;
    }

    void yield() override { vTaskDelay(1); }

private:
    int _sck, _miso, _mosi;
    spi_device_handle_t _spi = nullptr;
    bool _initialized = false;
};
