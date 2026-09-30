// INCLUDES -------------------------------------------------------------------------------------------------------------------
#include "Wire.h"
#include "MT6701.hpp" // https://github.com/noranraskin/MT6701


// GLOBALS --------------------------------------------------------------------------------------------------------------------
// define encoder objects
MT6701 enc1;
MT6701 enc2;
MT6701 enc3;
MT6701 enc4;

// encoder initialised values (zero points) <- SET MANUALLY
float enc1_zero = 5.51;
float enc2_zero = 5.31;
float enc3_zero = 0.51;
float enc4_zero = 3.28;

// encoder current values
float enc1_value = 0.0;
float enc2_value = 0.0;
float enc3_value = 0.0;
float enc4_value = 0.0;

// encoder current ouputs
float enc1_output = 0.0;
float enc2_output = 0.0;
float enc3_output = 0.0;
float enc4_output = 0.0;


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
  float enc_value = enc.getAngleRadians();
  return enc_value;
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
  enc1_value = getEncoderValue(enc1, 2);
  enc2_value = getEncoderValue(enc2, 3);
  enc3_value = getEncoderValue(enc3, 4);
  enc4_value = getEncoderValue(enc4, 5);

  // set output angles relative to zero points
  if (enc1_value >= enc1_zero) { // enc 1
    enc1_output = enc1_value - enc1_zero;
  } else {
    enc1_output = 2*PI - enc1_zero + enc1_value;
  }
  if (enc2_value >= enc2_zero) { // enc 2
    enc2_output = enc2_value - enc2_zero;
  } else {
    enc2_output = 2*PI - enc2_zero + enc2_value;
  }
  if (enc3_value >= enc3_zero) { // enc 3
    enc3_output = enc3_value - enc3_zero;
  } else {
    enc3_output = 2*PI - enc3_zero + enc3_value;
  }
  if (enc4_value >= enc4_zero) { // enc 4
    enc4_output = enc4_value - enc4_zero;
  } else {
    enc4_output = 2*PI - enc4_zero + enc4_value;
  }

  // print values in serial read format
  Serial.print("");
  Serial.print(enc1_output);
  Serial.print(",");
  Serial.print(enc2_output);
  Serial.print(",");
  Serial.print(enc3_output);
  Serial.print(",");
  Serial.println(enc4_output);
  
  delay(20);
}
