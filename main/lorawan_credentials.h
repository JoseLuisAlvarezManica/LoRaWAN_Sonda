#pragma once

/**
 * @file lorawan_credentials.h
 * @brief Credenciales OTAA para conexión LoRaWAN con Chirpstack
 *
 * INSTRUCCIONES DE REGISTRO EN CHIRPSTACK:
 *  1. Flashear el firmware con los valores por defecto (ceros).
 *  2. Abrir el monitor serie (idf.py monitor) y copiar los tres valores:
 *         DevEUI   — identificador único del dispositivo (generado desde MAC)
 *         AppEUI   — identificador de la aplicación (se puede dejar en ceros)
 *         AppKey   — clave de sesión (Chirpstack la genera al dar de alta el nodo)
 *  3. En Chirpstack:
 *       a. Crear (o seleccionar) una aplicación.
 *       b. Agregar un dispositivo con el DevEUI obtenido.
 *       c. Seleccionar perfil OTAA / LoRaWAN 1.0.x.
 *       d. Pegar el AppKey generado por Chirpstack en LORAWAN_APPKEY abajo.
 *  4. Reemplazar los bytes de LORAWAN_APPKEY y reflashear.
 *
 * NOTA: el DevEUI se calcula automáticamente al arrancar desde el MAC
 * del chip ESP32. No es necesario modificarlo aquí.
 */

/* ── JoinEUI / AppEUI (8 bytes, orden MSB primero) ─────────────────────────
 * En Chirpstack v4 con OTAA 1.0.x se puede dejar en ceros.
 * Ajustar si la red exige un AppEUI específico.                              */
#define LORAWAN_JOINEUI  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/* ── AppKey (16 bytes, orden MSB primero) ──────────────────────────────────
 * *** REEMPLAZAR con el AppKey generado por Chirpstack ***
 * Ejemplo de formato:
 *   { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
 *     0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }  */
#define LORAWAN_APPKEY                                                          \
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                          \
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/* ── Región de frecuencias ──────────────────────────────────────────────────
 * Definir SOLO UNA de las siguientes según la ubicación del dispositivo:
 *
 *   LORAWAN_REGION_US915  — Américas (México, EE.UU., Canadá)  ← predefecto
 *   LORAWAN_REGION_EU868  — Europa
 *   LORAWAN_REGION_AU915  — Australia
 *   LORAWAN_REGION_AS923  — Asia / Pacífico                                  */
#define LORAWAN_REGION_US915

/* ── Subband para US915 / AU915 ─────────────────────────────────────────────
 * Chirpstack Americas y TTN usan subband 2 (canales 8-15 + canal 65).
 * Dejar en 0 para EU868, AS923, etc.                                         */
#define LORAWAN_SUBBAND  2

/* ── Puerto de aplicación FPort ─────────────────────────────────────────────
 * Puerto LoRaWAN para los uplinks de sensores (1-223).                       */
#define LORAWAN_APP_PORT  1

/* ── Intervalo de envío en segundos ─────────────────────────────────────────
 * Respetar el duty cycle y el FUP de la red.
 * Mínimo recomendado: 60 s.  Para pruebas usar 60 s como mínimo.             */
#define LORAWAN_UPLINK_INTERVAL_S  60
