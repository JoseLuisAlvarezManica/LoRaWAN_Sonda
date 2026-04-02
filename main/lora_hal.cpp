/**
 * @file lora_hal.cpp
 * @brief Implementación del wrapper C para RadioLib SX1276
 *
 * Chip: SX1276 (Heltec WiFi LoRa 32 V2 estándar)
 * DIO0=26 → TxDone / RxDone IRQ.  Sin pin BUSY (SX1276 no lo tiene).
 *
 * Este archivo compila como C++ e instancia RadioLib internamente.
 * main.c lo llama a través de la interfaz C definida en lora_hal.h
 * sin saber que hay C++ por dentro.
 */

#include "lora_hal.h"
#include "lorawan_credentials.h"
#include "EspHal.h"
#include "RadioLib.h"
#include "esp_log.h"
#include "esp_mac.h"

static const char *TAG = "LORA_HAL";

static EspHal *hal   = nullptr;
static Module *mod   = nullptr;
static SX1276 *radio = nullptr;
static int _dio0 = RADIOLIB_NC;
static int _nss  = RADIOLIB_NC;
static int _rst  = RADIOLIB_NC;

/* ── lora_init ──────────────────────────────────────────────────────────── */

int lora_init(lora_config_t *config) {
    if (!config) return LORA_ERR_INIT;

    _dio0 = config->dio0_pin;
    _nss  = config->nss_pin;
    _rst  = config->rst_pin;

    // El HAL solo necesita SCK, MISO, MOSI — RadioLib gestiona CS por GPIO
    hal = new EspHal(config->sclk_pin, config->miso_pin, config->mosi_pin);

    // Module(hal, cs, irq/dio0, rst, gpio)
    // SX1276 no usa BUSY; gpio=RADIOLIB_NC (DIO1 no requerido para TX/RX básico)
    mod = new Module(hal,
                     config->nss_pin,
                     config->dio0_pin,
                     config->rst_pin,
                     RADIOLIB_NC);

    radio = new SX1276(mod);

    ESP_LOGI(TAG, "HAL creado — pines SCLK:%d MISO:%d MOSI:%d NSS:%d RST:%d DIO0:%d",
             config->sclk_pin, config->miso_pin, config->mosi_pin,
             config->nss_pin,  config->rst_pin,  config->dio0_pin);
    return LORA_OK;
}

/* ── lora_begin ─────────────────────────────────────────────────────────── */

int lora_begin(uint32_t frequency, int8_t power) {
    if (!radio) return LORA_ERR_INIT;

    float freq_mhz = (float)frequency / 1e6f;

    // SX1276: begin(freq, bw, sf, cr, syncWord, power, preambleLength, gain)
    // syncWord 0x12 = red privada por defecto; se sobreescribe con lora_set_sync_word() después.
    // gain=0 → AGC automático.
    int16_t state = radio->begin(freq_mhz, 125.0f, 7, 5,
                                 0x12,   /* private syncWord (SX127x default) */
                                 power, 8, 0);

    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "SX1276 listo — %.1f MHz  SF7  BW125  CR4/5  %d dBm",
                 freq_mhz, power);
        return LORA_OK;
    }

    ESP_LOGE(TAG, "SX1276 begin() falló: error %d", state);
    ESP_LOGE(TAG, "  -2 = chip no encontrado (CHIP_NOT_FOUND)");
    ESP_LOGE(TAG, "  Verificar pines: NSS=%d RST=%d DIO0=%d", _nss, _rst, _dio0);
    return LORA_ERR_INIT;
}

/* ── lora_set_sync_word ─────────────────────────────────────────────────── */

void lora_set_sync_word(uint8_t sync_word) {
    if (!radio) return;
    int16_t state = radio->setSyncWord(sync_word);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Error setSyncWord(0x%02X): %d", sync_word, state);
    } else {
        ESP_LOGI(TAG, "Sync word: 0x%02X", sync_word);
    }
}

/* ── lora_send ──────────────────────────────────────────────────────────── */

int lora_send(const uint8_t *data, size_t length) {
    if (!radio || !data || length == 0) return LORA_ERR_INIT;

    // transmit() es bloqueante: espera hasta que TxDone se señaliza en DIO0.
    int16_t state = radio->transmit(data, length);

    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "Paquete transmitido OK (%d bytes)", (int)length);
        return LORA_OK;
    }

    ESP_LOGE(TAG, "Error transmit(): %d", state);
    return LORA_ERR_INIT;
}

/* ── lora_start_receive ─────────────────────────────────────────────────── */

int lora_start_receive(void) {
    if (!radio) return LORA_ERR_INIT;
    int16_t state = radio->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Error startReceive(): %d", state);
        return LORA_ERR_INIT;
    }
    return LORA_OK;
}

/* ── lora_available ─────────────────────────────────────────────────────── */

int lora_available(void) {
    if (!hal || _dio0 == RADIOLIB_NC) return 0;
    // DIO0 se pone HIGH en RxDone cuando la radio está en modo RX continuo (SX1276)
    return hal->digitalRead(_dio0) ? 1 : 0;
}

/* ── lora_read ──────────────────────────────────────────────────────────── */

int lora_read(uint8_t *buffer, uint8_t max_length) {
    if (!radio || !buffer || max_length == 0) return 0;

    size_t pkt_len = radio->getPacketLength();
    if (pkt_len == 0) {
        radio->startReceive();
        return 0;
    }
    if (pkt_len > max_length) pkt_len = max_length;

    int16_t state = radio->readData(buffer, pkt_len);

    // Rearmar recepción continua siempre, independientemente del resultado
    radio->startReceive();

    if (state == RADIOLIB_ERR_NONE) {
        return (int)pkt_len;
    }
    if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        ESP_LOGW(TAG, "Paquete descartado: CRC error");
    } else {
        ESP_LOGE(TAG, "Error readData(): %d", state);
    }
    return 0;
}

/* ── lora_get_packet_status ─────────────────────────────────────────────── */

void lora_get_packet_status(int8_t *rssi, int8_t *snr) {
    if (!radio || !rssi || !snr) return;
    *rssi = (int8_t)radio->getRSSI();
    *snr  = (int8_t)radio->getSNR();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  LoRaWAN OTAA
 * ═══════════════════════════════════════════════════════════════════════════ */

static LoRaWANNode *lwNode = nullptr;

/* ── lorawan_get_deveui_from_mac ─────────────────────────────────────────── */

void lorawan_get_deveui_from_mac(uint8_t deveui[8]) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BASE);

    /* Conversión EUI-64: se insertan 0xFF 0xFE entre los bytes 2 y 3 del MAC */
    deveui[0] = mac[0];
    deveui[1] = mac[1];
    deveui[2] = mac[2];
    deveui[3] = 0xFF;
    deveui[4] = 0xFE;
    deveui[5] = mac[3];
    deveui[6] = mac[4];
    deveui[7] = mac[5];
}

/* ── lorawan_begin ───────────────────────────────────────────────────────── */

int lorawan_begin(const uint8_t joinEUI[8], const uint8_t devEUI[8],
                  const uint8_t appKey[16]) {
    if (!radio) {
        ESP_LOGE(TAG, "lorawan_begin: radio no inicializado (llamar lora_init primero)");
        return LORA_ERR_INIT;
    }

    /* Inicializar el chip SX1276 — LoRaWAN sobreescribirá freq/SF/BW */
    int16_t state = radio->begin();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "radio->begin() falló: %d", state);
        return LORA_ERR_INIT;
    }
    ESP_LOGI(TAG, "SX1276 inicializado para LoRaWAN");

    /* Seleccionar región y subband */
#if defined(LORAWAN_REGION_US915)
    lwNode = new LoRaWANNode(radio, &US915, LORAWAN_SUBBAND);
    ESP_LOGI(TAG, "Región: US915  Subband: %d", LORAWAN_SUBBAND);
#elif defined(LORAWAN_REGION_EU868)
    lwNode = new LoRaWANNode(radio, &EU868, 0);
    ESP_LOGI(TAG, "Región: EU868");
#elif defined(LORAWAN_REGION_AU915)
    lwNode = new LoRaWANNode(radio, &AU915, LORAWAN_SUBBAND);
    ESP_LOGI(TAG, "Región: AU915  Subband: %d", LORAWAN_SUBBAND);
#elif defined(LORAWAN_REGION_AS923)
    lwNode = new LoRaWANNode(radio, &AS923, 0);
    ESP_LOGI(TAG, "Región: AS923");
#else
#error "Definir una región en lorawan_credentials.h (p.ej. LORAWAN_REGION_US915)"
#endif

    /* Convertir arrays de bytes a uint64_t en orden MSB */
    uint64_t joinEUI_u64 = 0, devEUI_u64 = 0;
    for (int i = 0; i < 8; i++) {
        joinEUI_u64 = (joinEUI_u64 << 8) | joinEUI[i];
        devEUI_u64  = (devEUI_u64  << 8) | devEUI[i];
    }

    /* LoRaWAN 1.0.x: nwkKey == nullptr → appKey se usa como root key */
    state = lwNode->beginOTAA(joinEUI_u64, devEUI_u64, nullptr,
                              const_cast<uint8_t *>(appKey));
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "beginOTAA falló: %d", state);
        lwNode = nullptr;
        return LORA_ERR_INIT;
    }

    ESP_LOGI(TAG, "Nodo LoRaWAN OTAA configurado correctamente");
    return LORA_OK;
}

/* ── lorawan_join_otaa ───────────────────────────────────────────────────── */

int lorawan_join_otaa(void) {
    if (!lwNode) {
        ESP_LOGE(TAG, "lorawan_join_otaa: nodo no inicializado");
        return LORA_ERR_INIT;
    }

    ESP_LOGI(TAG, "Ejecutando Join OTAA...");
    int16_t state = lwNode->activateOTAA();

    if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
        ESP_LOGI(TAG, "Join OTAA exitoso — sesión nueva establecida");
        goto join_ok;
    }
    if (state == RADIOLIB_LORAWAN_SESSION_RESTORED) {
        ESP_LOGI(TAG, "Join OTAA exitoso — sesión restaurada");
        goto join_ok;
    }

    ESP_LOGE(TAG, "Join OTAA falló: estado RadioLib = %d", state);
    if (state == -1116) {
        ESP_LOGE(TAG, "  → RADIOLIB_ERR_NO_JOIN_ACCEPT");
        ESP_LOGE(TAG, "  Causas comunes:");
        ESP_LOGE(TAG, "  1. AppKey no guardada/activada en Chirpstack.");
        ESP_LOGE(TAG, "  2. DevNonce repetido — en Chirpstack: Device > OTAA keys > Reset.");
        ESP_LOGE(TAG, "  3. Subband incorrecto — gateway y nodo deben usar subband 2 (US915).");
        ESP_LOGE(TAG, "  4. El gateway no escucha los canales 8-15 (subband 2).");
    }
    return LORA_ERR_INIT;

join_ok:
    /* US915 DR0 = SF10/BW125 → máx 11 bytes. Nuestro payload ocupa 20 bytes,
       por lo que forzamos DR1 (SF9/BW125 → máx 53 bytes).                   */
    {
        int16_t dr_state = lwNode->setDatarate(1);   /* DR1 */
        if (dr_state != RADIOLIB_ERR_NONE) {
            ESP_LOGW(TAG, "setDatarate(DR1) falló: %d — se usará el DR por defecto", dr_state);
        } else {
            ESP_LOGI(TAG, "Data rate fijado a DR1 (SF9/BW125, máx 53 bytes)");
        }
    }
    return LORA_OK;
}

/* ── lorawan_send_uplink ─────────────────────────────────────────────────── */

int lorawan_send_uplink(const uint8_t *data, size_t length, uint8_t port) {
    if (!lwNode || !data || length == 0) {
        ESP_LOGE(TAG, "lorawan_send_uplink: parámetros inválidos");
        return LORA_ERR_INIT;
    }

    int16_t state = lwNode->sendReceive(const_cast<uint8_t *>(data),
                                        length, port);

    if (state >= RADIOLIB_ERR_NONE) {
        if (state > 0) {
            ESP_LOGI(TAG, "Uplink OK — downlink recibido en ventana Rx%d", state);
        } else {
            ESP_LOGI(TAG, "Uplink OK — sin downlink");
        }
        return LORA_OK;
    }

    ESP_LOGE(TAG, "sendReceive falló: %d", state);
    return LORA_ERR_INIT;
}
