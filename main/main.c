/**
 * @file main.c
 * @brief Nodo LoRaWAN OTAA con lectura de sensor via Modbus RTU
 *
 * Lee datos reales de un sensor agricola de suelo via Modbus RTU (RS485)
 * y los transmite a una red LoRaWAN (Chirpstack) usando OTAA.
 *
 * Al arrancar imprime en consola DevEUI, AppEUI y AppKey para registrar
 * el dispositivo en Chirpstack antes del primer join.
 *
 * Flujo de operacion:
 *  1. Imprimir credenciales OTAA en consola.
 *  2. Inicializar radio SX1276 y configurar nodo LoRaWAN.
 *  3. Inicializar Modbus RTU master (UART1, RS485 half-duplex).
 *  4. Ejecutar Join OTAA con reintentos cada 15 s hasta conectar.
 *  5. Cada LORAWAN_UPLINK_INTERVAL_S segundos:
 *       a. Leer 7 registros Modbus del sensor (slave addr=1, reg 0-6).
 *       b. Empaquetar en payload binario de 20 bytes.
 *       c. Enviar uplink LoRaWAN (FPort LORAWAN_APP_PORT).
 *
 * Formato del payload (20 bytes, big-endian):
 *  [0:1]   temperatura   int16  valor x 100  (grados C)
 *  [2:3]   humedad       int16  valor x 100  (%)
 *  [4:5]   nitrogeno     int16  valor x 100  (mg/kg)
 *  [6:7]   fosforo       int16  valor x 100  (mg/kg)
 *  [8:9]   potasio       int16  valor x 100  (mg/kg)
 *  [10:11] ph            int16  valor x 100
 *  [12:13] conductividad int16  valor x 100  (uS/cm)
 *  [14:19] MAC           6 bytes - direccion MAC base del chip (ESP_MAC_BASE)
 *
 * Mapeo de registros Modbus del sensor (Function Code 03):
 *  Reg 0 -> Humedad      (raw / 10.0)
 *  Reg 1 -> Temperatura  (raw / 10.0)
 *  Reg 2 -> Conductividad EC (raw, uS/cm)
 *  Reg 3 -> pH           (raw / 10.0)
 *  Reg 4 -> Nitrogeno    (raw, mg/kg)
 *  Reg 5 -> Fosforo      (raw, mg/kg)
 *  Reg 6 -> Potasio      (raw, mg/kg)
 *
 * Hardware: Heltec WiFi LoRa 32 V2 (SX1276)
 *   SPI: SCLK=5  MISO=19  MOSI=27  NSS=18  RST=14  DIO0=26
 * RS485: TX=17  RX=22  RE/DE=23  UART1  4800 baud  8N1
 */

#include "lora_hal.h"
#include "lorawan_credentials.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include <math.h>

#include "mbcontroller.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "LORAWAN_MODBUS";

/* -- Pines SPI Heltec WiFi LoRa 32 V2 -------------------------------------- */
#define PIN_MISO  19
#define PIN_MOSI  27
#define PIN_SCLK   5
#define PIN_NSS   18
#define PIN_RST   14
#define PIN_DIO0  26

/* -- Pines RS485 / Modbus --------------------------------------------------- */
#define RE_DE_PIN    23   /* Control de flujo (DE y RE puenteados) */
#define RX_PIN       22
#define TX_PIN       17
#define MB_PORT_NUM  UART_NUM_1

/* -- Ultimos valores leidos del sensor -------------------------------------- */
static float lastHum  = 0, lastTemp = 0, lastEc = 0, lastPh = 0;
static float lastN    = 0, lastP    = 0, lastK  = 0;
static bool  lastReadSuccess = false;
static void *master_handle   = NULL;

/* -- Escribe un int16 big-endian en dos bytes consecutivos del buffer ------- */
static inline void put_be16(uint8_t *buf, int16_t val) {
    buf[0] = (uint8_t)((uint16_t)val >> 8);
    buf[1] = (uint8_t)(val & 0xFF);
}

/* -- Imprime DevEUI / AppEUI / AppKey en consola para registro en Chirpstack */
static void imprimir_credenciales_lorawan(const uint8_t deveui[8],
                                          const uint8_t joineui[8],
                                          const uint8_t appkey[16]) {
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "  DATOS DE REGISTRO EN CHIRPSTACK");
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "  DevEUI : %02X%02X%02X%02X%02X%02X%02X%02X",
             deveui[0], deveui[1], deveui[2], deveui[3],
             deveui[4], deveui[5], deveui[6], deveui[7]);
    ESP_LOGI(TAG, "  AppEUI : %02X%02X%02X%02X%02X%02X%02X%02X",
             joineui[0], joineui[1], joineui[2], joineui[3],
             joineui[4], joineui[5], joineui[6], joineui[7]);
    ESP_LOGI(TAG, "  AppKey : %02X%02X%02X%02X%02X%02X%02X%02X"
                             "%02X%02X%02X%02X%02X%02X%02X%02X",
             appkey[0],  appkey[1],  appkey[2],  appkey[3],
             appkey[4],  appkey[5],  appkey[6],  appkey[7],
             appkey[8],  appkey[9],  appkey[10], appkey[11],
             appkey[12], appkey[13], appkey[14], appkey[15]);
    ESP_LOGI(TAG, "------------------------------------------------");
    ESP_LOGI(TAG, "  1. Copiar DevEUI/AppEUI/AppKey en Chirpstack.");
    ESP_LOGI(TAG, "  2. Pegar el AppKey generado por Chirpstack en");
    ESP_LOGI(TAG, "     lorawan_credentials.h y reflashear el nodo.");
    ESP_LOGI(TAG, "================================================");
}

/* -- Inicializa el master Modbus RTU sobre UART1 / RS485 -------------------- */
static esp_err_t inicializar_modbus_master(void) {
    mb_communication_info_t comm = {
        .ser_opts.port             = MB_PORT_NUM,
        .ser_opts.mode             = MB_RTU,
        .ser_opts.baudrate         = 4800,
        .ser_opts.parity           = UART_PARITY_DISABLE,
        .ser_opts.uid              = 0,
        .ser_opts.response_tout_ms = 1000,
    };

    esp_err_t err = mbc_master_create_serial(&comm, &master_handle);
    if (err != ESP_OK) return err;

    err = uart_set_pin(MB_PORT_NUM, TX_PIN, RX_PIN, RE_DE_PIN, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = mbc_master_start(master_handle);
    if (err != ESP_OK) return err;

    /* RS485 half-duplex: el transceiver conmuta automaticamente con RE/DE */
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);

    ESP_LOGI(TAG, "Modbus Master OK - UART%d  TX:%d  RX:%d  RE/DE:%d  4800 baud",
             MB_PORT_NUM, TX_PIN, RX_PIN, RE_DE_PIN);
    return err;
}

/* -- Lee 7 registros Modbus del sensor y actualiza las variables globales --- */
static void leer_sensor_modbus(void) {
    if (!master_handle) return;

    /* Lectura 1: Humedad, Temperatura, EC, pH  (reg 0x0000-0x0003) */
    mb_param_request_t req1 = {
        .slave_addr = 1,
        .command    = 3,
        .reg_start  = 0x0000,
        .reg_size   = 4
    };
    uint16_t data1[4] = {0};
    esp_err_t err1 = mbc_master_send_request(master_handle, &req1, (void *)data1);
    if (err1 != ESP_OK) {
        lastReadSuccess = false;
        ESP_LOGE(TAG, "Error leyendo Modbus (T/H/EC/pH): %s", esp_err_to_name(err1));
        return;
    }
    lastHum  = (float)data1[0] / 10.0f;
    lastTemp = (float)data1[1] / 10.0f;
    lastEc   = (float)data1[2];
    lastPh   = (float)data1[3] / 10.0f;

    /* Pausa entre tramas: algunos sensores necesitan tiempo entre solicitudes */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Lectura 2: Nitrogeno, Fosforo, Potasio  (reg 0x0004-0x0006)
     * Segun datasheet ZTS-3001: reg4=N, reg5=P, reg6=K (mg/kg, sin escalar)
     * Se lee en trama separada para evitar truncamiento de respuesta de 7 registros */
    mb_param_request_t req2 = {
        .slave_addr = 1,
        .command    = 3,
        .reg_start  = 0x0004,
        .reg_size   = 3
    };
    uint16_t data2[3] = {0};
    esp_err_t err2 = mbc_master_send_request(master_handle, &req2, (void *)data2);
    if (err2 != ESP_OK) {
        lastReadSuccess = false;
        ESP_LOGE(TAG, "Error leyendo Modbus (NPK): %s", esp_err_to_name(err2));
        return;
    }
    lastN = (float)data2[0];  /* mg/kg */
    lastP = (float)data2[1];
    lastK = (float)data2[2];

    lastReadSuccess = true;
    ESP_LOGI(TAG, "Modbus OK - T:%.1f C  H:%.1f%%  EC:%.0f uS/cm  pH:%.1f"
                  "  N:%.0f  P:%.0f  K:%.0f mg/kg",
             lastTemp, lastHum, lastEc, lastPh, lastN, lastP, lastK);
}

/* -- Empaqueta los valores del sensor en 20 bytes big-endian (payload LoRaWAN)
 *
 *  Bytes  Campo         Tipo    Escala     Unidad
 *  [0:1]  temperatura   int16   x100       grados C
 *  [2:3]  humedad       int16   x100       %
 *  [4:5]  nitrogeno     int16   x100       mg/kg
 *  [6:7]  fosforo       int16   x100       mg/kg
 *  [8:9]  potasio       int16   x100       mg/kg
 *  [10:11]ph            int16   x100       -
 *  [12:13]conductividad int16   x100       uS/cm
 *  [14:19]MAC           bytes   -          direccion MAC base del chip
 * --------------------------------------------------------------------------- */
static void empaquetar_sensores(uint8_t payload[20]) {
    put_be16(&payload[0],  (int16_t)roundf(lastTemp * 100.0f));
    put_be16(&payload[2],  (int16_t)roundf(lastHum  * 100.0f));
    put_be16(&payload[4],  (int16_t)roundf(lastN    * 100.0f));
    put_be16(&payload[6],  (int16_t)roundf(lastP    * 100.0f));
    put_be16(&payload[8],  (int16_t)roundf(lastK    * 100.0f));
    put_be16(&payload[10], (int16_t)roundf(lastPh   * 100.0f));
    put_be16(&payload[12], (int16_t)roundf(lastEc   * 100.0f));
    esp_read_mac(&payload[14], ESP_MAC_BASE);

    ESP_LOGI(TAG, "  T:%.2f C  H:%.2f%%  N:%.2f  P:%.2f  K:%.2f"
                  "  pH:%.2f  CE:%.2f  MAC:%02X:%02X:%02X:%02X:%02X:%02X",
             lastTemp, lastHum, lastN, lastP, lastK, lastPh, lastEc,
             payload[14], payload[15], payload[16],
             payload[17], payload[18], payload[19]);
}

/* -- Tarea FreeRTOS: ciclo periodico de lectura Modbus + uplink LoRaWAN ----- */
static void tarea_lorawan(void *pvParameters) {
    uint32_t contador = 0;
    uint8_t  payload[20];

    while (1) {
        /* Esperar el intervalo configurado antes de cada uplink */
        vTaskDelay(pdMS_TO_TICKS((uint32_t)LORAWAN_UPLINK_INTERVAL_S * 1000UL));

        contador++;
        ESP_LOGI(TAG, "==== Uplink #%lu ====", contador);

        /* 1. Leer sensor */
        leer_sensor_modbus();

        /* 2. Omitir el uplink si la lectura Modbus fallo */
        if (!lastReadSuccess) {
            ESP_LOGW(TAG, "Lectura Modbus fallida - uplink omitido en ciclo #%lu.",
                     contador);
            continue;
        }

        /* 3. Empaquetar y transmitir con hasta 3 intentos */
        empaquetar_sensores(payload);

        int ret = LORA_ERR_INIT;
        for (int intento = 1; intento <= 3; intento++) {
            ret = lorawan_send_uplink(payload, 20, LORAWAN_APP_PORT);
            if (ret == LORA_OK) {
                ESP_LOGI(TAG, "Uplink #%lu enviado correctamente (intento %d).",
                         contador, intento);
                break;
            }
            ESP_LOGW(TAG, "Uplink #%lu fallo (intento %d/3).", contador, intento);
            if (intento < 3) {
                vTaskDelay(pdMS_TO_TICKS(15000)); /* esperar 15 s antes de reintentar */
            }
        }

        /* 4. Si los 3 intentos fallaron, hacer re-join OTAA y reenviar */
        if (ret != LORA_OK) {
            ESP_LOGE(TAG, "Uplink #%lu fallo 3 veces. Iniciando re-join OTAA...",
                     contador);
            int join_ret;
            int join_intento = 0;
            do {
                join_intento++;
                join_ret = lorawan_join_otaa();
                if (join_ret == LORA_OK) {
                    ESP_LOGI(TAG, "Re-join OTAA exitoso (intento %d).", join_intento);
                } else {
                    ESP_LOGW(TAG, "Re-join fallo (intento %d). Reintentando en 15 s...",
                             join_intento);
                    vTaskDelay(pdMS_TO_TICKS(15000));
                }
            } while (join_ret != LORA_OK);

            /* Reenviar el uplink pendiente tras reconectarse */
            ESP_LOGI(TAG, "Reenviando uplink #%lu tras reconexion...", contador);
            ret = lorawan_send_uplink(payload, 20, LORAWAN_APP_PORT);
            if (ret == LORA_OK) {
                ESP_LOGI(TAG, "Uplink #%lu reenviado correctamente.", contador);
            } else {
                ESP_LOGE(TAG, "Uplink #%lu fallo incluso tras re-join.", contador);
            }
        }
    }
}

/* -- app_main --------------------------------------------------------------- */
void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Nodo LoRaWAN  -  Sensor Modbus RTU");
    ESP_LOGI(TAG, " Hardware : Heltec WiFi LoRa 32 V2");
    ESP_LOGI(TAG, " IDF      : %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");

    /* Inicializar NVS (requerido para esp_read_mac y RadioLib session store) */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* Derivar DevEUI desde MAC del chip y cargar credenciales compiladas */
    uint8_t deveui[8];
    lorawan_get_deveui_from_mac(deveui);

    const uint8_t joineui[8] = LORAWAN_JOINEUI;
    const uint8_t appkey[16] = LORAWAN_APPKEY;

    /* Mostrar credenciales para registro manual en Chirpstack */
    imprimir_credenciales_lorawan(deveui, joineui, appkey);
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Inicializar el bus SPI y el chip SX1276 */
    lora_config_t radio_cfg = {
        .miso_pin = PIN_MISO,
        .mosi_pin = PIN_MOSI,
        .sclk_pin = PIN_SCLK,
        .nss_pin  = PIN_NSS,
        .rst_pin  = PIN_RST,
        .dio0_pin = PIN_DIO0,
    };

    if (lora_init(&radio_cfg) != LORA_OK) {
        ESP_LOGE(TAG, "Error critico: no se pudo inicializar la radio.");
        esp_restart();
    }

    /* Configurar nodo OTAA (region, subband, claves) */
    if (lorawan_begin(joineui, deveui, appkey) != LORA_OK) {
        ESP_LOGE(TAG, "Error critico: lorawan_begin fallo.");
        esp_restart();
    }

    /* Inicializar Modbus (fallo no critico: el uplink simplemente se omite) */
    esp_err_t mb_ret = inicializar_modbus_master();
    if (mb_ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando Modbus (%s) - continuando sin sensor.",
                 esp_err_to_name(mb_ret));
    }

    /* Join OTAA con reintentos indefinidos cada 15 s */
    ESP_LOGI(TAG, "Iniciando Join OTAA...");
    int join_intentos = 0;
    while (lorawan_join_otaa() != LORA_OK) {
        join_intentos++;
        ESP_LOGW(TAG, "Join fallo (intento %d). Reintentando en 15 s...",
                 join_intentos);
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
    ESP_LOGI(TAG, "Conectado a la red LoRaWAN. Iniciando uplinks...");

    /* Lanzar la tarea periodica de uplink */
    xTaskCreate(
        tarea_lorawan,
        "lorawan_uplink",
        8192,
        NULL,
        5,
        NULL
    );

    ESP_LOGI(TAG, "Primer uplink en %d s.", LORAWAN_UPLINK_INTERVAL_S);
}