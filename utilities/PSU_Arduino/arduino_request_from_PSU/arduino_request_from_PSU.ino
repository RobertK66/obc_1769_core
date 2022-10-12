//MASTER


#include <Wire.h>

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Serial.begin(9600);
}

void loop() {

 int reg_num = 3; // access register from which we would like to read data from
 int PSU_address = 0b1010101; // address of PSU 
 int datavector;
    
  Wire.beginTransmission(PSU_address); // transmit to device #4
  Wire.write(reg_num);
  Wire.endTransmission();    // stop transmitting



 Wire.requestFrom(PSU_address,3); // address and amount of bytes we expect to receive
  while (Wire.available()) {
    datavector = Wire.read();
    Serial.print(datavector);
  }
  Serial.println();
  delay(1000);

 
}
