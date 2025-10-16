/**
 * server.cpp  —  UAV-side ESP32 bridge
 * ------------------------------------
 * Forwards JSON data from the Jetson to the UGV via ESP-NOW.
 * 
 * Responsibilities:
 *   • Read JSON lines from Jetson UART (USB or GPIO serial)
 *   • Send those packets directly to paired ESP32 on UGV
 *   • Print acknowledgments for debugging
 * 
 * JSON packets are newline-terminated and must not be reformatted.
 * Includes transmission pacing to support future system scalability.
 */

#include <WiFi.h>
#include <esp_now.h>

// === CONFIGURATION ===
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 50      // milliseconds
#define TX_DELAY_MS 500        // pacing delay between transmissions
const uint8_t UGV_MAC[] = {0x88, 0x13, 0xBF, 0x07, 0xAD, 0xC0}; // <-- replace with your actual UGV ESP32 MAC

// === GLOBALS ===
String buffer = "";

// === ESP-NOW SEND CALLBACK ===
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW] Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// === SETUP ===
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_TIMEOUT);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    while (true) { delay(1000); }
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, UGV_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Failed to add peer!");
    while (true) { delay(1000); }
  }

  esp_now_register_send_cb(onSend);
  Serial.println("[INFO] ESP-NOW initialized, ready to transmit.");
}

// === LOOP ===
void loop() {
  if (Serial.available()) {
    buffer = Serial.readStringUntil('\n');   // Jetson sends newline-terminated JSON
    buffer.trim();

    if (buffer.length() > 0) {
      esp_err_t result = esp_now_send(UGV_MAC, (uint8_t *)buffer.c_str(), buffer.length());
      if (result == ESP_OK) {
        Serial.print("[TX] ");
        Serial.println(buffer);
      } else {
        Serial.println("[ERROR] ESP-NOW send failed!");
      }

      // Controlled pacing delay for system stability and future expandability
      delay(TX_DELAY_MS);
    }
  }
}
