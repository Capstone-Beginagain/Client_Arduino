#include <Arduino_LSM6DS3.h>

void setup() {

  Serial.begin(9600);

  if (!IMU.begin()) { //LSM6DS3센서 시작

    Serial.println("LSM6DS3센서 오류!");

    while (1);

  }

}

float accel_x, accel_y, accel_z;

float gyro_x, gyro_y, gyro_z;

void loop() {

  // IMU.accelerationAvailable()와 IMU.gyroscopeAvailable() 
  // 조건을 함께 체크하여 두 센서의 데이터가 모두 준비되었을 때만 데이터를 읽습니다.

 if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    Serial.print("{\"ax\":");
    Serial.print(accel_x);
    Serial.print(", \"ay\":");
    Serial.print(accel_y);
    Serial.print(", \"az\":");
    Serial.print(accel_z);
    Serial.print(", \"gx\":");
    Serial.print(gyro_x);
    Serial.print(", \"gy\":");
    Serial.print(gyro_y);
    Serial.print(", \"gz\":");
    Serial.print(gyro_z);
    Serial.println("}");

    delay(100);
  }
  

}