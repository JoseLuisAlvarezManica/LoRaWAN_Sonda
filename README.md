# Nodo LoRaWAN OTAA + Sensor Agrícola (Modbus RS485) — ESP32

## Descripción

Firmware ESP-IDF para un nodo LoRaWAN con **ahorro de batería mediante deep sleep**. Lee datos de un **sensor agrícola de suelo** (7 en 1: Humedad, Temperatura, Conductividad EC, pH, Nitrógeno, Fósforo, Potasio) mediante **Modbus RTU sobre RS485**, y los transmite a una red **LoRaWAN Chirpstack** usando **OTAA (Over-the-Air Activation)**. Entre ciclos, el ESP32 entra en **deep sleep** (~10 µA) para minimizar el consumo de batería.

Al arrancar, el nodo imprime por consola serie los valores **DevEUI, AppEUI y AppKey** necesarios para dar de alta el dispositivo en Chirpstack. Una vez registrado y con el AppKey correcto, el nodo ejecuta el Join OTAA de forma automática con reintentos, envía el uplink y entra en deep sleep hasta el siguiente ciclo.

### Características principales

- **Deep sleep entre ciclos**: El ESP32 entra en deep sleep por `LORAWAN_UPLINK_INTERVAL_S` segundos tras cada uplink. El timer RTC despierta el chip, que reinicia y realiza un nuevo join OTAA. Consumo típico en sleep: ~10 µA.
- **LoRaWAN OTAA**: Activación segura con sesión cifrada. Compatible con Chirpstack v4, TTN y otras redes LoRaWAN 1.0.x.
- **Lectura Modbus RTU**: Usa el componente oficial `espressif/esp-modbus` para comunicarse con el sensor a 4800 baud por UART1/RS485.
- **Sensor alimentado por transistor**: El sensor RS485 se enciende solo durante la lectura Modbus y se apaga inmediatamente después, eliminando su consumo en reposo.
- **Payload binario de 20 bytes**: Empaquetado big-endian compacto (sin JSON) para maximizar el tiempo en aire y cumplir con los límites de duty cycle LoRaWAN.
- **MAC embebida en el payload**: Los últimos 6 bytes del uplink incluyen la dirección MAC base del chip para identificar el nodo en el servidor de aplicaciones.
- **Contador persistente en RTC memory**: El número de uplink enviados sobrevive el deep sleep y se resetea solo al cortar la alimentación.
- **Reintentos automáticos**: Si el Join OTAA falla, reintenta cada 15 s indefinidamente. Si una lectura Modbus falla, el uplink se omite ese ciclo y el dispositivo entra igualmente en deep sleep.

### Flujo de operación por ciclo

```
Despertar (timer RTC) / Encendido inicial
  │
  ├─ Imprimir credenciales OTAA
  ├─ Inicializar radio SX1276
  ├─ Inicializar Modbus RTU master
  ├─ Join OTAA (reintentos cada 15 s hasta conectar)
  ├─ Encender sensor → Leer Modbus → Apagar sensor
  ├─ Empaquetar payload (20 bytes)
  ├─ Enviar uplink LoRaWAN (hasta 3 intentos)
  └─ Deep sleep (LORAWAN_UPLINK_INTERVAL_S segundos)
```

## Consumo de Energía

| Estado              | Consumo típico |
|---------------------|----------------|
| Deep sleep          | ~10 µA         |
| Join OTAA + uplink  | ~80–240 mA     |
| Sensor RS485 activo | ~20–50 mA      |

El consumo promedio depende de `LORAWAN_UPLINK_INTERVAL_S` (actualmente **1800 s / 30 min**). Con un ciclo activo de ~10–15 s y deep sleep el resto del tiempo, el consumo promedio es inferior a 1 mA, lo que permite alimentar el nodo con baterías pequeñas durante meses.

### Manejo de GPIO durante Deep Sleep

Durante el deep sleep, los GPIOs quedan **flotando** (sin control del procesador), lo que puede mantener accidentalmente activos componentes como el transistor de control del sensor RS485. Para evitar esto, el firmware utiliza:

- **`gpio_hold_en(SENSOR_PWR_PIN)`** antes de entrar en deep sleep: congela GPIO 13 en su nivel actual (bajo, apagando el sensor).
- **`gpio_hold_dis(SENSOR_PWR_PIN)`** al despertar: libera el hold para que el GPIO pueda ser controlado nuevamente en el siguiente ciclo.

Esto garantiza que el sensor permanece desconectado durante toda la duración del deep sleep, minimizando el consumo de batería.

> **Nota:** El deep sleep reinicia el chip. El nodo realiza un Join OTAA completo en cada ciclo; no se persiste la sesión LoRaWAN entre reinicios.

## Hardware Soportado

### Componentes
- **Microcontrolador**: Heltec WiFi LoRa 32 V2 (ESP32 + SX1276 integrado).
- **Transceptor LoRa**: SX1276 (interno en Heltec V2), 915 MHz banda US915.
- **Interfaz RS485**: Módulo MAX485 o similar (DE y RE puenteados al mismo GPIO).
- **Sensor**: Sensor agrícola de suelo RS485, 7 en 1 (Modbus RTU, slave addr 1).

### Conexiones

#### SPI — Transceptor LoRa SX1276 (interno en Heltec V2)
| Señal    | GPIO | Descripción                          |
|----------|------|--------------------------------------|
| SPI SCK  |  5   | Reloj SPI                            |
| SPI MISO | 19   | Master In Slave Out                  |
| SPI MOSI | 27   | Master Out Slave In                  |
| NSS / CS | 18   | Chip Select                          |
| RST      | 14   | Reset del módulo                     |
| DIO0     | 26   | IRQ — TxDone / RxDone                |

#### UART1 — Modbus RTU / RS485
| Señal  | GPIO | Descripción                                       |
|--------|------|---------------------------------------------------|
| TX     |  17  | Conectar al pin DI (Driver In) del módulo RS485   |
| RX     |  22  | Conectar al pin RO (Receiver Out) del módulo RS485|
| RE / DE|  23  | Control de flujo (DE y RE puenteados en el módulo)|

#### Control de alimentación del sensor
| Señal       | GPIO | Descripción                                            |
|-------------|------|--------------------------------------------------------|
| SENSOR_PWR  |  13  | Control de transistor (activo en alto). Alimenta el sensor RS485 solo durante la lectura Modbus. Durante deep sleep, GPIO 13 se congela en bajo mediante gpio_hold_en(). |

## Formato del Payload LoRaWAN (20 bytes, big-endian)

| Bytes  | Campo         | Tipo   | Escala | Ejemplo decodificado         | Unidad |
|--------|---------------|--------|--------|------------------------------|--------|
| 0–1    | Temperatura   | int16  | ×10    | `0x00F5` → 24.5 °C           | °C     |
| 2–3    | Humedad       | int16  | ×10    | `0x0258` → 60.0 %            | %      |
| 4–5    | Nitrógeno     | uint16 | ×1     | `0x0064` → 100 mg/kg         | mg/kg  |
| 6–7    | Fósforo       | uint16 | ×1     | `0x0032` → 50 mg/kg          | mg/kg  |
| 8–9    | Potasio       | uint16 | ×1     | `0x00C8` → 200 mg/kg         | mg/kg  |
| 10–11  | pH            | int16  | ×10    | `0x0049` → 7.3               | —      |
| 12–13  | Conductividad | uint16 | ×1     | `0x01F4` → 500 µS/cm         | µS/cm  |
| 14–19  | MAC           | bytes  | —      | dirección MAC base del chip  | —      |

**Ejemplo de decodificación (Python/ChirpStack codec):**
```python
def decode(fport, bytes):
    def s16(b, i): return int.from_bytes(b[i:i+2], 'big', signed=True) / 100
    def u16(b, i): return int.from_bytes(b[i:i+2], 'big', signed=False)
    return {
        "temperatura":   s16(bytes, 0),
        "humedad":       s16(bytes, 2),
        "nitrogeno":     u16(bytes, 4)  / 100,
        "fosforo":       u16(bytes, 6)  / 100,
        "potasio":       u16(bytes, 8)  / 100,
        "ph":            s16(bytes, 10),
        "conductividad": u16(bytes, 12) / 10,   # escala x10, max 6553.5 uS/cm
        "mac": ':'.join(f'{b:02X}' for b in bytes[14:20])
    }
```

## Primeros Pasos — Registro en Chirpstack

1. **Flashear** el firmware con `LORAWAN_APPKEY` en ceros (valores por defecto).
2. **Abrir el monitor serie** (`idf.py monitor`) y copiar los tres valores que imprime al arrancar:
   - `DevEUI` — identificador único derivado del MAC del chip.
   - `AppEUI` — normalmente `0000000000000000`.
   - `AppKey` — se usará el generado por Chirpstack en el paso siguiente.
3. **En Chirpstack**:
   - Crear o seleccionar una aplicación.
   - Agregar un dispositivo con el DevEUI copiado.
   - Seleccionar perfil **OTAA / LoRaWAN 1.0.x**.
   - Copiar el **AppKey generado por Chirpstack**.
4. **Editar** `main/lorawan_credentials.h` y reemplazar `LORAWAN_APPKEY` con el valor de Chirpstack.
5. **Reflashear** el firmware. El nodo ejecutará el Join OTAA, enviará el primer uplink y entrará en deep sleep automáticamente.

> **Nota sobre deep sleep:** Como el nodo hace un nuevo join OTAA tras cada ciclo de deep sleep, asegúrate de que en Chirpstack el perfil de dispositivo **no requiera contador de frames estricto** o permita re-joins frecuentes.

## Dependencias

Declaradas en `main/idf_component.yml`:

| Componente                 | Versión       | Función                        |
|----------------------------|---------------|--------------------------------|
| `jgromes/radiolib`         | `^7.6.0`      | Driver SX1276 + stack LoRaWAN  |
| `espressif/esp-modbus`     | `*` (latest)  | Stack Modbus RTU master        |

## Compilación y Flasheo

```bash
# 1. Configurar entorno ESP-IDF (si no lo está ya)
. $IDF_PATH/export.sh   # Linux/macOS
# o bien usar ESP-IDF PowerShell en Windows

# 2. Instalar componentes (primera vez)
idf.py reconfigure

# 3. Compilar
idf.py build

# 4. Flashear y abrir monitor
idf.py -p COM3 flash monitor
# Sustituir COM3 por el puerto correcto (Linux: /dev/ttyUSB0)
```

## Configuración Rápida

Los parámetros más comunes se encuentran en dos archivos:

| Archivo                        | Qué configura                                      |
|--------------------------------|----------------------------------------------------|
| `main/lorawan_credentials.h`   | AppKey, JoinEUI, región (US915/EU868), subband, FPort, intervalo de uplink (y por tanto de deep sleep) |
| `main/main.c`                  | Pines SPI/RS485/sensor, dirección slave, registros Modbus |

## Registros Modbus del Sensor (Function Code 03, slave addr 1)

| Registro | Campo         | Escala    | Unidad |
|----------|---------------|-----------|--------|
| 0        | Humedad       | raw ÷ 10  | %      |
| 1        | Temperatura   | raw ÷ 10  | °C     |
| 2        | Conductividad | raw       | µS/cm  |
| 3        | pH            | raw ÷ 10  | —      |
| 4        | Nitrógeno     | raw       | mg/kg  |
| 5        | Fósforo       | raw       | mg/kg  |
| 6        | Potasio       | raw       | mg/kg  |
