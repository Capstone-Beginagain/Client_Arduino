#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>

// WiFi 네트워크 정보
const char* ssid = "your_SSID";       // WiFi 이름
const char* password = "your_PASSWORD"; // WiFi 비밀번호

// UDP 통신 설정
WiFiUDP udp;
const char* remoteIp = "192.168.0.100"; // 자바 서버 IP 주소
const int remotePort = 12345;           // 자바 서버에서 수신 대기할 포트
const int localPort = 12344;            // 아두이노에서 사용할 포트

void setup() {
  Serial.begin(115200);
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;

  // WiFi 연결
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // UDP 초기화
  udp.begin(localPort);
  Serial.println("UDP initialized.");
}

void loop() {

  // UDP 메시지 수신
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0'; // 문자열 종료
    }
    Serial.print("Received packet: ");
    Serial.println(incomingPacket);
  }
  
  //자이로센서 값
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
    String jsonMessage = "{";
    jsonMessage += "\"ax\":" + String(accel_x, 3) + ",";
    jsonMessage += "\"ay\":" + String(accel_y, 3) + ",";
    jsonMessage += "\"az\":" + String(accel_z, 3) + ",";
    jsonMessage += "\"gx\":" + String(gyro_x, 3) + ",";
    jsonMessage += "\"gy\":" + String(gyro_y, 3) + ",";
    jsonMessage += "\"gz\":" + String(gyro_z, 3);
    jsonMessage += "}";
    
    //json 형식{"ax":0.123,"ay":-0.456,"az":0.789,"gx":1.234,"gy":-2.345,"gz":3.456}

    //UDP 송신
    udp.beginPacket(remoteIp, remotePort);
    udp.print(jsonMessage);
    udp.endPacket();
    Serial.println("Message sent: " + jsonMessage);

  }

  delay(2000); // 2초 대기
}
