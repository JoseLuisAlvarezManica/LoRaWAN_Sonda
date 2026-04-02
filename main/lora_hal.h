#pragma once

/**
 * @brief Interfaz C para el módulo LoRa SX1276 (implementado con RadioLib)
 *
 * Hardware objetivo: Heltec WiFi LoRa 32 V2 con SX1276
 *   SCLK=5, MISO=19, MOSI=27, NSS=18, RST=14, DIO0=26
 *   (SX1276 no tiene pin BUSY)
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ── Códigos de retorno ─────────────────────────────────────────────────── */
#define LORA_OK         0
#define LORA_ERR_INIT  -1

/* ── Configuración de pines ──────────────────────────────────────────────── */
typedef struct {
    int miso_pin;
    int mosi_pin;
    int sclk_pin;
    int nss_pin;
    int rst_pin;
    int dio0_pin;   /* IRQ/TxDone+RxDone — GPIO26 en Heltec V2 */
    /* SX1276 no tiene pin BUSY */
} lora_config_t;

typedef lora_config_t sx1276_config_t;

/* ── Funciones principales ───────────────────────────────────────────────── */

/** Inicializa el bus SPI y los GPIOs. */
int  lora_init(lora_config_t *config);

/** Configura la radio (frecuencia en Hz, potencia en dBm). */
int  lora_begin(uint32_t frequency, int8_t power);

/** Configura el sync word (formato SX127x de 1 byte, RadioLib convierte). */
void lora_set_sync_word(uint8_t sync_word);

/** Transmite un paquete LoRa. Espera a que termine la transmisión.
 *  Retorna LORA_OK (0) si fue exitoso, LORA_ERR_INIT (-1) si falló. */
int  lora_send(const uint8_t *data, size_t length);

/** Pone la radio en modo recepción continua. */
int  lora_start_receive(void);

/** Retorna 1 si hay un paquete disponible (DIO0 alto). */
int  lora_available(void);

/** Lee el paquete disponible en buffer. Retorna bytes leídos, 0 si error. */
int  lora_read(uint8_t *buffer, uint8_t max_length);

/** Obtiene RSSI y SNR del último paquete recibido. */
void lora_get_packet_status(int8_t *rssi, int8_t *snr);

/* ── LoRaWAN OTAA ────────────────────────────────────────────────────────────
 * Flujo de uso:
 *   1. lora_init(config)                         — crea el objeto radio
 *   2. lorawan_begin(joinEUI, devEUI, appKey)     — configura nodo OTAA
 *   3. lorawan_join_otaa()                        — join a la red (bloqueante)
 *   4. lorawan_send_uplink(data, len, port)       — uplink periódico
 */

/** Calcula el DevEUI de 8 bytes (EUI-64) a partir del MAC del ESP32.
 *  Formato resultante: MAC[0..2] | 0xFF | 0xFE | MAC[3..5] (MSB first).     */
void lorawan_get_deveui_from_mac(uint8_t deveui[8]);

/** Inicializa el nodo LoRaWAN OTAA.
 *  Llama internamente a radio->begin() y node->beginOTAA().
 *  Debe llamarse DESPUÉS de lora_init().                                      */
int lorawan_begin(const uint8_t joinEUI[8], const uint8_t devEUI[8],
                  const uint8_t appKey[16]);

/** Ejecuta el procedimiento de Join por OTAA.  Bloqueante.
 *  Retorna LORA_OK si el join resultó exitoso.                                */
int lorawan_join_otaa(void);

/** Transmite un uplink LoRaWAN y abre las ventanas de recepción.
 *  port: FPort de aplicación (1-223).
 *  Retorna LORA_OK si el uplink fue aceptado.                                 */
int lorawan_send_uplink(const uint8_t *data, size_t length, uint8_t port);

#ifdef __cplusplus
}
#endif
