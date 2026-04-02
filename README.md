# Nodo LoRaWAN OTAA + Sensor Agrícola (Modbus RS485) — ESP32

## Descripción

Firmware ESP-IDF para un nodo LoRaWAN que lee datos reales de un **sensor agrícola de suelo** (7 en 1: Humedad, Temperatura, Conductividad EC, pH, Nitrógeno, Fósforo, Potasio) mediante **Modbus RTU sobre RS485**, y los transmite a una red **LoRaWAN Chirpstack** usando el procedimiento de activación **OTAA (Over-the-Air Activation)**.

Al arrancar, el nodo imprime por consola serie los valores **DevEUI, AppEUI y AppKey** necesarios para dar de alta el dispositivo en Chirpstack. Una vez registrado y con el AppKey correcto, el nodo ejecuta el Join OTAA de forma automática con reintentos, y después envía uplinks periódicos en binario compacto.

### Características principales

- **LoRaWAN OTAA**: Activación segura con sesión cifrada. Compatible con Chirpstack v4, TTN y otras redes LoRaWAN 1.0.x.
- **Lectura Modbus RTU**: Usa el componente oficial `espressif/esp-modbus` para comunicarse con el sensor a 4800 baud por UART1/RS485.
- **Payload binario de 20 bytes**: Empaquetado big-endian compacto (sin JSON) para maximizar el tiempo en aire y cumplir con los límites de duty cycle LoRaWAN.
- **MAC embebida en el payload**: Los últimos 6 bytes del uplink incluyen la dirección MAC base del chip para identificar el nodo en el servidor de aplicaciones.
- **Reintentos automáticos**: Si el Join OTAA falla, reintenta cada 30 s indefinidamente. Si una lectura Modbus falla, el uplink se omite ese ciclo sin detener el sistema.
- **FreeRTOS**: La lógica de uplink corre en una tarea dedicada; `app_main` termina después del join.

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

## Formato del Payload LoRaWAN (20 bytes, big-endian)

| Bytes  | Campo         | Tipo  | Escala | Unidad |
|--------|---------------|-------|--------|--------|
| 0–1    | Temperatura   | int16 | ×100   | °C     |
| 2–3    | Humedad       | int16 | ×100   | %      |
| 4–5    | Nitrógeno     | int16 | ×100   | mg/kg  |
| 6–7    | Fósforo       | int16 | ×100   | mg/kg  |
| 8–9    | Potasio       | int16 | ×100   | mg/kg  |
| 10–11  | pH            | int16 | ×100   | —      |
| 12–13  | Conductividad | int16 | ×100   | µS/cm  |
| 14–19  | MAC           | bytes | —      | dirección MAC base del chip |

**Ejemplo de decodificación (Python/ChirpStack codec):**
```python
def decode(fport, bytes):
    def be16(b, i): return int.from_bytes(b[i:i+2], 'big', signed=True) / 100
    return {
        "temperatura":   be16(bytes, 0),
        "humedad":       be16(bytes, 2),
        "nitrogeno":     be16(bytes, 4),
        "fosforo":       be16(bytes, 6),
        "potasio":       be16(bytes, 8),
        "ph":            be16(bytes, 10),
        "conductividad": be16(bytes, 12),
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
5. **Reflashear** el firmware. El nodo ejecutará el Join OTAA automáticamente.

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
| `main/lorawan_credentials.h`   | AppKey, JoinEUI, región (US915/EU868), subband, FPort, intervalo de uplink |
| `main/main.c`                  | Pines SPI/RS485, dirección slave, registros Modbus |

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
