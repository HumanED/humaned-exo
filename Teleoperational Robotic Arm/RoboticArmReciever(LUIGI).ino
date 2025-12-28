#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_PWMServoDriver.h>

// I2C pins (Elegoo ESP32 typical defaults)
#define SDA_PIN 21
#define SCL_PIN 22

// PCA9685 I2C address (change if you use a different address)
#define PCA_ADDR 0x40

// Create PCA driver (uses Wire)
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA_ADDR);

// PCA / servo config
const uint16_t PCA_MIN = 102;   // pulse for 0 deg (tweak for your servos)
const uint16_t PCA_MAX = 512;   // pulse for 180 deg (tweak)
const uint16_t PCA_FREQ = 50;   // servo frequency in Hz

// Number of motors/encoders
const int N_MOTORS = 4;

// Safety angle limits (degrees) - same for all motors here
const float ANGLE_MIN = 45.0f;
const float ANGLE_MAX = 135.0f;

// Data structure — must match sender exactly
typedef struct struct_message {
  uint16_t raw[4];
  float    angleDeg[4];
} struct_message;

struct_message myData;
float angle[N_MOTORS] = {0,0,0,0}; // last received angles (degrees)
float angle_clamped[N_MOTORS] = {0,0,0,0}; // clamped values actually sent to motors

// Helper: print MAC
void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; ++i) {
    if (i) Serial.print(":");
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
  }
  Serial.println();
}

// Correct receive callback signature for current ESP32 cores
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  // Defensive copy: only copy up to the size of our struct
  size_t copySize = min((size_t)len, sizeof(myData));
  memcpy(&myData, incomingData, copySize);

  // If sender info is available, print source MAC
  Serial.print("Received ");
  Serial.print(len);
  Serial.print(" bytes from ");
  if (info && info->src_addr) printMac(info->src_addr);
  else Serial.println("unknown sender");

  // Update angle[] and print for debug
  for (int i = 0; i < N_MOTORS; ++i) {
    angle[i] = myData.angleDeg[i];
    // Clamp to safety limits
    if (!isfinite(angle[i])) angle[i] = 0.0f;
    angle_clamped[i] = angle[i];
    if (angle_clamped[i] < ANGLE_MIN) angle_clamped[i] = ANGLE_MIN;
    if (angle_clamped[i] > ANGLE_MAX) angle_clamped[i] = ANGLE_MAX;

    Serial.print("ch");
    Serial.print(i);
    Serial.print(" raw_angle=");
    Serial.print(angle[i], 3);
    Serial.print(" deg   clamped=");
    Serial.print(angle_clamped[i], 3);
    Serial.println(" deg");
  }
  Serial.println();
}

void setup() {
  // Serial first
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n=== Receiver: ESP-NOW -> PCA9685 motor control (with angle limits) ===");

  // Initialise I2C on pins SDA_PIN, SCL_PIN BEFORE PCA driver
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(10);
  Serial.print("I2C started on SDA=");
  Serial.print(SDA_PIN);
  Serial.print(" SCL=");
  Serial.println(SCL_PIN);

  // Init PCA9685
  pca.begin();
  pca.setPWMFreq(PCA_FREQ); // 50 Hz typical for servos
  delay(10);
  Serial.print("PCA9685 init at addr 0x");
  Serial.println(PCA_ADDR, HEX);

  // Setup WiFi + ESP-NOW
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  Serial.print("Local MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(1000);
  }
  // register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // initialise motors to a safe midpoint inside the allowed range
  float mid = (ANGLE_MIN + ANGLE_MAX) * 0.5f;
  for (int i = 0; i < N_MOTORS; ++i) {
    float pulse_f = ((mid / 180.0f) * (float)(PCA_MAX - PCA_MIN)) + (float)PCA_MIN;
    uint16_t pulse = (uint16_t)constrain((int)round(pulse_f), PCA_MIN, PCA_MAX);
    pca.setPWM(i, 0, pulse);
    angle[i] = mid;
    angle_clamped[i] = mid;
  }
  Serial.println("Setup complete. Motors initialised to safe midpoint.");
}

void loop() {
  // Convert clamped angles to PCA pulses and write
  for (int i = 0; i < N_MOTORS; ++i) {
    float a = angle_clamped[i];

    // Map float angle (0..180) to pulse width (PCA_MIN..PCA_MAX)
    float pulse_f = ((a / 180.0f) * (float)(PCA_MAX - PCA_MIN)) + (float)PCA_MIN;
    uint16_t pulse = (uint16_t)constrain((int)round(pulse_f), PCA_MIN, PCA_MAX);

    // apply to PCA channel i
    pca.setPWM(i, 0, pulse);
  }

  // small delay — update rate depends on your use-case
  delay(20); // 50 Hz update loop
}
