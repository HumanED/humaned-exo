#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define MT6701_ADDR 0x06
#define REG_ANGLE_H 0x03
#define REG_ANGLE_L 0x04
#define SDA_PIN 21
#define SCL_PIN 22

#define MUX_ADDR 0x70   // TCA9548A

uint8_t receiverMac[] = {0x14, 0x08, 0x08, 0xA5, 0xAF, 0xA0};

typedef struct struct_message {
  uint16_t raw[4];
  float    angleDeg[4];
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// ---------- TCA9548A ----------
void selectChannel(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(500);
}

bool readRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MT6701_ADDR, 1) != 1) return false;
  value = Wire.read();
  return true;
}

bool readMT6701AngleOnChannel(uint8_t ch, uint16_t &raw14, float &angleDeg) {
  selectChannel(ch);

  uint8_t hi, lo;
  if (!readRegister(REG_ANGLE_H, hi)) return false;
  if (!readRegister(REG_ANGLE_L, lo)) return false;

  raw14 = ((uint16_t)hi << 6) | (uint16_t)(lo >> 2);
  angleDeg = (raw14 * 360.0f) / 16384.0f;
  return true;
}

// ---------- ESP-NOW ----------
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("MT6701 x4 via TCA9548A ESP-NOW sender");

  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  Serial.print("Local MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init error");
    while (1) delay(1000);
  }
  esp_now_register_send_cb(OnDataSent);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1) delay(1000);
  }
}

void loop() {
  // Start line with timestamp/header
  Serial.print("[");

  // Read all 4 channels and print on same line
  for (uint8_t ch = 0; ch < 4; ch++) {
    uint16_t raw;
    float angle;

    if (!readMT6701AngleOnChannel(ch, raw, angle)) {
      myData.raw[ch] = 0;
      myData.angleDeg[ch] = 0.0f;
      Serial.print("ERR");
    } else {
      myData.raw[ch] = raw;
      myData.angleDeg[ch] = angle;
      Serial.printf("CH%d:%04X(%.1f)", ch, raw, angle);
    }

    if (ch < 3) Serial.print(" | ");
  }

  Serial.print("] ");

  // Send all 4 in one packet
  esp_err_t res = esp_now_send(receiverMac, (uint8_t *)&myData, sizeof(myData));
  if (res != ESP_OK) {
    Serial.print("SEND_ERR:");
    Serial.print((int)res);
  } else {
    Serial.print("SEND_OK");
  }

  Serial.println();  // Newline for next reading
  delay(50);         // ~20Hz update rate
}
