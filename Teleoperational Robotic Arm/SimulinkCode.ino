#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();
const uint8_t SERVOS[4] = {0, 1, 2, 3};
#define PULSE_MIN  102
#define PULSE_MAX  512

// SAFETY BOUNDARIES
#define MIN_SAFE_ANGLE 45.0
#define MAX_SAFE_ANGLE 135.0

union FloatBytes { float f; uint8_t b[4]; };
FloatBytes u;

// LAST VALID ANGLES - hold position if invalid command
float lastValidAngles[4] = {90.0, 90.0, 90.0, 90.0};  // default 90°

void setup() {
  Serial.begin(115200, SERIAL_8N1);
  pca.begin();
  pca.setPWMFreq(50);
  delay(10);
  
  // Initialize servos to safe middle position
  for (int i = 0; i < 4; i++) {
    uint16_t pulse = map(90, 0, 180, PULSE_MIN, PULSE_MAX);
    pca.setPWM(SERVOS[i], 0, pulse);
  }
}

void loop() {
  if (Serial.available() >= 20) {
    float angles[4];
    
    // Read 4 angles
    for (int i = 0; i < 4; i++) {
      angles[i] = readFromSimulink();
      
      // SAFETY CHECK: Only update if within bounds [45,135]
      if (angles[i] >= MIN_SAFE_ANGLE && angles[i] <= MAX_SAFE_ANGLE) {
        lastValidAngles[i] = angles[i];  // Update memory
      }
      // else: keep lastValidAngles[i] unchanged
    }
    
    // Apply LAST VALID angles to servos (always safe)
    for (int i = 0; i < 4; i++) {
      uint16_t pulse = map((int)lastValidAngles[i], 0, 180, PULSE_MIN, PULSE_MAX);
      pca.setPWM(SERVOS[i], 0, pulse);
    }
    
    // Echo back LAST VALID angles
    for (int i = 0; i < 4; i++) {
      writeToSimulink(lastValidAngles[i]);
    }
  }
}

float readFromSimulink() {
  for (int i = 0; i < 4; i++) {
    while (!Serial.available()) {}
    u.b[i] = Serial.read();
  }
  if (Serial.peek() == 13) Serial.read();
  if (Serial.peek() == 10) Serial.read();
  return u.f;
}

void writeToSimulink(float number) {
  FloatBytes out; out.f = number;
  Serial.write(out.b, 4);
  Serial.write(13); Serial.write(10);
}
