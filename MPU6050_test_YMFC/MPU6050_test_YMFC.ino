#include <Wire.h>

int cal_int;
unsigned long UL_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

void setup(){
  Wire.begin();
  Serial.begin(9600);

  /*Wire.beginTransmission(0x68);
  Wire.write(0x20);
  Wire.write(0x0F);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x23);
  Wire.write(0x10);
  Wire.endTransmission();*/

  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission();

  delay(250);

  Serial.print("Starting Calibration...");
  for(cal_int = 0; cal_int < 2000; cal_int ++){
    gyro_signalen();
    
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;

    if(cal_int%100 == 0)Serial.print(".");
    delay(4);
    }

    Serial.println("done!");

    gyro_roll_cal /= 2000;
    gyro_pitch_cal /= 2000;
    gyro_yaw_cal /= 2000;
    
  }

void loop(){
   delay(250);
   gyro_signalen();
   print_output();
  }
  
void gyro_signalen(){
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  
  while(Wire.available()<6);
  lowByte = Wire.read();
  highByte = Wire.read();
  gyro_roll = (highByte<<(8)|lowByte);
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal;
  lowByte = Wire.read();
  highByte = Wire.read();
  gyro_pitch = (highByte<<(8)|lowByte);
  gyro_pitch *= -1;
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;
  lowByte = Wire.read();
  highByte = Wire.read();
  gyro_yaw = (highByte<<(8)|lowByte);
  gyro_yaw *= -1;
  if(cal_int == 2000)gyro_yaw -=gyro_yaw_cal;
  }

void print_output(){
  
  Serial.print("Pitch:");
  if(gyro_pitch>=0)Serial.print("+");
  Serial.print(gyro_pitch/57.14286,0);
  if(gyro_pitch/57.14286-2>0)Serial.print("NoU");
  else if(gyro_pitch/57.14286+2<0)Serial.print("NoD");
  else Serial.print("---");
  
  Serial.print("Roll:");
  if(gyro_roll>=0)Serial.print("+");
  Serial.print(gyro_roll/57.14286,0);
  if(gyro_roll/57.14286-2>0)Serial.print("NwU");
  else if(gyro_roll/57.14286+2<0)Serial.print("NwD");
  else Serial.print("---");

  Serial.print("Yaw:");
  if(gyro_yaw>=0)Serial.print("+");
  Serial.print(gyro_yaw/57.14286,0);
  if(gyro_yaw/57.14286-2>0)Serial.print("NoR");
  else if(gyro_yaw/57.14286+2<0)Serial.print("NoL");
  else Serial.print("---");
  Serial.print("\n");
  }
