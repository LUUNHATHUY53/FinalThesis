#include <Wire.h>

unsigned long timer_channel_1;
unsigned long timer_1;
unsigned long loop_timer;
int esc_1 = 1650;
int throttle, battery_voltage;


void setup(){
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();                                                    //Set the timer for the next loop.    
 }

void loop(){
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);	  //Compensate the esc-1 pulse for voltage drop.
  }
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B00010000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
  }
}