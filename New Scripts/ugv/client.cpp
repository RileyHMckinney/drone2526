/**
 * client.cpp  —  UGV-side ESP32 bridge
 * ------------------------------------
 * Receives JSON packets from the UAV ESP32 via ESP-NOW
 * and forwards them to the Raspberry Pi over UART.
 *
 * Responsibilities:
 *   • Initialize ESP-NOW receiver
 *   • Print and forward incoming JSON messages to Raspberry Pi
 *   • Maintain pacing and readability for future expansion
 */

#include <WiFi.h>
#include <esp_now.h>

// === CONFIGURATION ===
#define SERIAL_BAUD 115200
#define FORWARD_DELAY_MS 500   // pacing delay to mirror UAV rate (safe for scaling)

// === GLOBALS ===
String dataFromUAV = "";

// === CALLBACK: when data is received via ESP-NOW ===
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  char receivedData[len + 1];
  memcpy(receivedData, incomingData, len);
  receivedData[len] = '\0';  // Null-terminate
  dataFromUAV = String(receivedData);

  // Print to local serial monitor for debugging
  Serial.print("[RX] ");
  Serial.println(dataFromUAV);

  // Forward directly to Raspberry Pi via UART
  Serial2.println(dataFromUAV);  // Assuming Serial2 connected to Pi UART RX
}

// === SETUP ===
void setup() {
  // USB Serial (debug) + GPIO Serial (Pi)
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, 16, 17); // TX=17, RX=16 (adjust for your wiring)

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    while (true) { delay(1000); }
  }

  // Register receive callback
  esp_now_register_recv_cb(onDataReceive);
  Serial.println("[INFO] ESP-NOW client initialized, waiting for data...");
}

// === LOOP ===
void loop() {
  // All work handled in callback; pacing here helps stability for future code layers
  delay(FORWARD_DELAY_MS);
}
