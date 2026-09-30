// INCLUDES -------------------------------------------------------------------------------------------------------------------
#include "Wire.h"
#include "MT6701.hpp" // https://github.com/noranraskin/MT6701


// GLOBALS --------------------------------------------------------------------------------------------------------------------
// define encoder objects
MT6701 enc1;
MT6701 enc2;
MT6701 enc3;
MT6701 enc4;

// encoder outputs
float enc1_read_radians = 0.0;
float enc2_read_radians = 0.0;
float enc3_read_radians = 0.0;
float enc4_read_radians = 0.0;


// SCRIPT ---------------------------------------------------------------------------------------------------------------------
// I2C bus selection function
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// encoder value get function from bus
float getEncoderValue(MT6701 &enc, int bus){
  TCA9548A(bus);
  enc.updateCount();
  float enc_read_radians = enc.getAngleRadians();
  return enc_read_radians;
}

void setup() {
  Serial.begin(115200);

  // start I2C communication with TCA9548A multiplexer
  Wire.begin();

  // init encoder 1 on bus 2
  TCA9548A(2);
  enc1.begin();
  // init encoder 2 on bus 3
  TCA9548A(3);
  enc2.begin();
  // init encoder 3 on bus 4
  TCA9548A(4);
  enc3.begin();
  // init encoder 4 on bus 5
  TCA9548A(5);
  enc4.begin();
}

void loop() {
  // read encoder values
  enc1_read_radians = getEncoderValue(enc1, 2);
  enc2_read_radians = getEncoderValue(enc2, 3);
  enc3_read_radians = getEncoderValue(enc3, 4);
  enc4_read_radians = getEncoderValue(enc4, 5);

  // print values in readable format
  Serial.print("1: ");
  Serial.print(enc1_read_radians);
  Serial.print(",\t2: ");
  Serial.print(enc2_read_radians);
  Serial.print(",\t3: ");
  Serial.print(enc3_read_radians);
  Serial.print(",\t4: ");
  Serial.println(enc4_read_radians);
  
  delay(128);
}
