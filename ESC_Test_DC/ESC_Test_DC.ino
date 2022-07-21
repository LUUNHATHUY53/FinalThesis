#include <Wire.h>

void setup(){
  Serial.begin(9600);
  Wire.begin();
  //TWBR = 12;
  
  DDRD |= B11110000;
  analogWrite(5, 191);
  //pinMode(4, OUTPUT);
 }
 
void loop(){
    digitalWrite(4, HIGH);
    delayMicroseconds(600);
    digitalWrite(4, LOW);
    delayMicroseconds(300);
    
}
