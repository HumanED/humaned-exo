#include "MT6701.hpp"
#include <ESP32Servo.h>

// Wiring:
//  MT6701 -> ESP32:
//    SDA -> D21
//    SCL -> D22
//  Servo: D5


// ENCODERS AND JOINTS ---------------------------------------
// encoder 1 and joint 1
Servo joint1;
MT6701 enc1;
float enc1_zero_degrees = 0.0;
float enc1_read_degrees = 0.0;
float enc1_output_degrees = 0.0;


// SCRIPT ----------------------------------------------------
void setup() {
    Serial.begin(115200);
    joint1.attach(5);
    enc1.begin();
}

void loop() {
    enc1_read_degrees = enc1.getAngleDegrees();
    if (enc1_zero_degrees == 0.0) {
      enc1_zero_degrees = enc1_read_degrees;
    }

    if (enc1_read_degrees >= enc1_zero_degrees) {
      enc1_output_degrees = enc1_read_degrees - enc1_zero_degrees;
    } else {
      enc1_output_degrees = 360 - enc1_zero_degrees + enc1_read_degrees;
    }

    Serial.print("zero: ");
    Serial.print(enc1_zero_degrees);
    Serial.print(", read: ");
    Serial.print(enc1_read_degrees);
    Serial.print(", output: ");
    Serial.println(enc1_output_degrees);

    if (enc1_output_degrees <= 180) {
      joint1.write(enc1_output_degrees);
    }

    delay(64);
}