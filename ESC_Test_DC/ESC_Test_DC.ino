#include <Wire.h>

void setup(){
  Serial.begin(9600);
  Wire.begin();
  //TWBR = 12;
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  //DDRD |= B11110000;
  analogWrite(5, 191);
  pinMode(6, OUTPUT);
 }
 
void loop(){
    digitalWrite(6, HIGH);
    delayMicroseconds(600);
    digitalWrite(6, LOW);
    delayMicroseconds(900);
    
}
