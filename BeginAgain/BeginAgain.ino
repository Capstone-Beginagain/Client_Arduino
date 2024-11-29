#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>

int status = WL_IDLE_STATUS;
char ssid[] = "TestSon";  // Wi-Fi SSID
char pass[] = "00012340"; // Wi-Fi 비밀번호
unsigned int localPort = 9877;   // 아두이노의 UDP 수신 포트
unsigned int serverPort = 9877; // 자바 서버의 포트
//9877: 왼손, 9878: 오른손, 9879:왼발, 9880: 오른발
//IPAddress ip(192,168,195,220);  // 아두이노의 고정 IP
IPAddress serverIP(192,168,195,183); // 자바 서버의 IP 
WiFiUDP Udp; // WiFiUDP 객체 생성

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
void setup() {
  Serial.begin(57600);
  while (!Serial) {
    ; // 시리얼 포트 준비 대기
  }

  delay(3000);
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware!");
  }

  //WiFi.config(ip); // 고정 IP 설정
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(1000);
  }

  Serial.println("WiFi connected.");
  

  Serial.println("\nStarting UDP connection...");
  Udp.begin(localPort); // UDP 포트 시작
  if (!IMU.begin()) { //LSM6DS3센서 시작

    Serial.println("LSM6DS3센서 오류!");

    while (1);

  }
}

void loop() {
  // WiFi 연결 상태 확인
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    while (status != WL_CONNECTED) {
      status = WiFi.begin(ssid, pass);
      delay(1000);
    }
    Serial.println("WiFi reconnected.");
    Udp.begin(localPort); // UDP 포트 재시작
  }
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z); 
    // Serial.print("{\"ax\":");
    // Serial.print(accel_x);
    // Serial.print(", \"ay\":");
    // Serial.print(accel_y);
    // Serial.print(", \"az\":");
    // Serial.print(accel_z);
    // Serial.print(", \"gx\":");
    // Serial.print(gyro_x);
    // Serial.print(", \"gy\":");
    // Serial.print(gyro_y);
    // Serial.print(", \"gz\":");
    // Serial.print(gyro_z);
    // Serial.println("}");
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
    Udp.beginPacket(serverIP, serverPort);
    Udp.print(jsonMessage);
    Udp.endPacket();
    Serial.println("Message sent: " + jsonMessage);
    delay(500);

  }

  delay(500);
}


