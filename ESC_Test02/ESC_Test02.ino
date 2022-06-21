#include <Wire.h>

void setup(){
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;    
 }
 
void loop(){
  analogWrite(4, 127);
  delay(10);
}