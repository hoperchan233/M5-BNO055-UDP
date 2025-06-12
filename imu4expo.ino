#include <M5StickCPlus2.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// 若需串口调试输出，取消注释
#define DEBUG_SERIAL

// 若需屏显，取消注释
#define ENABLE_DISPLAY

// WiFi & UDP 配置
const char* SSID      = "free-wifi";
const char* PASSWORD  = "free-wifi";
const char* UDP_ADDR  = "192.168.19.8";
const uint16_t UDP_PORT = 8887;

// 背光控制管脚
const uint8_t BACKLIGHT_PIN = 27;
bool backlightOn = true;

// BNO055 初始化
Adafruit_BNO055 bno(55);
WiFiUDP udp;

// 用于记录上一次的航向角
float prevHeading = 0;

void setup() {
  #ifdef DEBUG_SERIAL
    Serial.begin(115200);
    Serial.println("Setup start");
  #endif

  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextSize(2);

  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);
  #ifdef DEBUG_SERIAL
    Serial.println("Backlight ON (default)");
  #endif

  // 连接 WiFi
  WiFi.begin(SSID, PASSWORD);
  #ifdef DEBUG_SERIAL
    Serial.print("Connecting to WiFi: "); Serial.println(SSID);
  #endif
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef DEBUG_SERIAL
      Serial.print(".");
    #endif
  }
  #ifdef DEBUG_SERIAL
    Serial.println("\nWiFi connected");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
  #endif

  // 初始化 BNO055
  if (!bno.begin()) {
    #ifdef DEBUG_SERIAL
      Serial.println("BNO055 init error");
    #endif
    M5.Lcd.println("BNO055 Err");
    while (1) delay(1000);
  }
  #ifdef DEBUG_SERIAL
    Serial.println("BNO055 init OK");
  #endif
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);
  #ifdef DEBUG_SERIAL
    Serial.println("BNO055 set to NDOF mode");
  #endif

  // 读取初始航向角，作为后续差分计算的基准
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    prevHeading = euler.x();  // x() 即 heading（航向角），单位：度
    #ifdef DEBUG_SERIAL
      Serial.printf("Initial Heading: %.2f°\n", prevHeading);
    #endif
  }

  #ifdef ENABLE_DISPLAY
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.println("IMU Ready");
  #endif
}

void loop() {
  M5.update();

  if (M5.BtnA.wasPressed()) {
    backlightOn = !backlightOn;
    digitalWrite(BACKLIGHT_PIN, backlightOn ? HIGH : LOW);
    #ifdef DEBUG_SERIAL
      Serial.printf("Backlight %s\n", backlightOn ? "ON" : "OFF");
    #endif
  }
  if (M5.BtnB.wasPressed()) {
    #ifdef DEBUG_SERIAL
      Serial.println("Button B pressed: restarting");
    #endif
    esp_restart();
  }

  // 读取线性加速度和陀螺仪角速度
  auto la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  auto av = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // 读取当前航向角（heading），并计算相对于上次循环的差值
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float currHeading = euler.x();             // 当前航向角 0–360°
  float deltaHeading = currHeading - prevHeading;
  // 对差值进行 ±180° 范围映射，保证 CW 为正，CCW 为负
  if (deltaHeading > 180.0f)      deltaHeading -= 360.0f;
  else if (deltaHeading < -180.0f) deltaHeading += 360.0f;
  prevHeading = currHeading;        // 更新基准

  #ifdef DEBUG_SERIAL
    Serial.printf("Accel: x=%.2f y=%.2f z=%.2f\n", la.x(), la.y(), la.z());
    Serial.printf("Gyro : x=%.1f y=%.1f z=%.1f\n", av.x(), av.y(), av.z());
    Serial.printf("YawΔ  : %+ .1f°\n", deltaHeading);
  #endif

  #ifdef ENABLE_DISPLAY
    if (backlightOn) {
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.printf("Ax:%.2f Ay:%.2f Az:%.2f\n", la.x(), la.y(), la.z());
      M5.Lcd.printf("Rx:%.1f Ry:%.1f Rz:%.1f\n", av.x(), av.y(), av.z());
      M5.Lcd.printf("Yaw:%+ .1f\n", deltaHeading);
    }
  #endif

  // 构造 JSON，并将水平旋转量（YawΔ）加入
  String json = String("{\"Accel\":{\"x\":") + String(la.x(),2) +
                ",\"y\":" + String(la.y(),2) +
                ",\"z\":" + String(la.z(),2) + "}," +
                "\"AngVel\":{\"x\":" + String(av.x(),1) +
                ",\"y\":" + String(av.y(),1) +
                ",\"z\":" + String(av.z(),1) + "}," +
                "\"YawRot\":" + String(deltaHeading,1) + // drgee/s
                "}";
  udp.beginPacket(UDP_ADDR, UDP_PORT);
  udp.print(json);
  udp.endPacket();

  delay(100);
}
