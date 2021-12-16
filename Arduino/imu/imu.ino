#include "MPU6050.h"
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
  // состояние соединения
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
  delay(1000);
}
void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.println(mpu.getAngleZ());

  delay(5);
}
