/* Get tilt angles on X and Y, and rotation angle on Z
   Angles are given in degrees

   License: MIT
*/

#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

int getZ() {//0-360 градусов 
  int b;
  if ((int)mpu.getAngleZ() % 360 < 0)b = 360 + ((int)mpu.getAngleZ()%360);
  else b = (int)mpu.getAngleZ()%360;
  return b;
}

int getZz(){
//0-180 -180-0 градусов
  int b;
  if((int)mpu.getAngleZ() % 360<-180)b=360+((int)mpu.getAngleZ() % 360);
  else if((int)mpu.getAngleZ() % 360>180)b=360-((int)mpu.getAngleZ() % 360);
  else b=(int)mpu.getAngleZ() % 360;
  return b;
  }

void loop() {
 unsigned long a = millis();
 mpu.update();
 
  Serial.print(getZ()); 
   Serial.print(F("\t"));
 Serial.println(millis() - a);
}
