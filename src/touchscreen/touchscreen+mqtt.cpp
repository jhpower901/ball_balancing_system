// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// This demo code returns raw readings, public domain

#include <Arduino.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define UUID "esp32-balance-001"
#define VERSION "1.0.0"
#define TOPIC_PUB_HELLO "ballbalancer/hello"
#define TOPIC_PUB_STATUS "ballbalancer/status"
#define TOPIC_SUB_CMD "ballbalancer/cmd"

// ====== Wi-Fi & MQTT 설정 ======
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
const char *MQTT_HOST = "anzam.kr"; // 로컬이면 "192.168.x.x"
const uint16_t MQTT_PORT = 1883;


// ====== 전역 ======
WiFiClient espClient;
PubSubClient client(espClient);

#define LED_PIN 14

void wifiConnect();
void mqttReconnect();
void callback(char *topic, byte *payload, unsigned int length);

#define YP 2  // 3 must be an analog pin, use "An" notation!
#define XM 1  // 4 must be an analog pin, use "An" notation!
#define YM 41 // 1 can be a digital pin
#define XP 42 // 2 can be a digital pin

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 360);

struct TupleInt
{
   int16_t x, y;
};

struct TupleFloat {
   float x, y;
};

struct PID_const {
   float kp_x, ki_x, kd_x;
   float kp_y, ki_y, kd_y;
};

struct Hello {
   char device_id[100] = UUID;
   char firmware[10] = VERSION;
   struct PID_const pid_const = {
      0.55, 0.05, 0.275,
      0.35, 0.05, 0.16
   };
   struct {
      int roll = 0;
      int pitch = 0;
   } platform_pose;
   struct {
      int width = 260;
      int height = 200;
   } field_size;
} hello;

struct Status {
   TupleInt real_pose = {0, 0};
   TupleInt target_pose = {0, 0};
   TupleFloat error= {0.0, 0.0};
   TupleFloat platform_pose= {0.0, 0.0};
   PID_const pid_const = {
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0
   };
   TupleFloat joystick_val = {0.0, 0.0};
} status;

TupleInt point_buffer[3];

TupleInt getPoint()
{
   /*
    * p : 150 - 890 사이의 값에서 안정
    * actual_pose : x: -130~130, y:-100~100
    */
   point_buffer[0] = point_buffer[1];
   point_buffer[1] = point_buffer[2];
   TupleInt p, actual_pose;
   TSPoint tp = ts.getPoint();
   if (tp.z > 0)
   {
      p.x = tp.x;
      p.y = tp.y;
   }
   else
   {
      p.x = point_buffer[1].x + (point_buffer[1].x - point_buffer[0].x);
      p.y = point_buffer[1].y + (point_buffer[1].y - point_buffer[0].y);
   }
   if (p.x < 0 || p.x > 1023)
   {
      p.x = point_buffer[1].x;
   }
   if (p.y < 0 || p.y > 1023)
   {
      p.y = point_buffer[1].y;
   }
   point_buffer[2] = p;
   actual_pose.x = map(p.x, 0, 1023, -130, 130);
   actual_pose.y = map(p.y, 0, 1023, -100, 100) - 9;
   return actual_pose;
}


String helloToJson(const Hello& h) {
  StaticJsonDocument<512> doc;

  doc["device_id"] = h.device_id;
  doc["firmware"] = h.firmware;

  JsonObject pid = doc.createNestedObject("pid_const");
  pid["kp_x"] = h.pid_const.kp_x;
  pid["ki_x"] = h.pid_const.ki_x;
  pid["kd_x"] = h.pid_const.kd_x;
  pid["kp_y"] = h.pid_const.kp_y;
  pid["ki_y"] = h.pid_const.ki_y;
  pid["kd_y"] = h.pid_const.kd_y;

  JsonObject pose = doc.createNestedObject("platform_pose");
  pose["roll"]  = h.platform_pose.roll;
  pose["pitch"] = h.platform_pose.pitch;

  JsonObject fs = doc.createNestedObject("field_size");
  fs["width"]  = h.field_size.width;
  fs["height"] = h.field_size.height;

  String output;
  serializeJson(doc, output);
  return output;
}

String statusToJson(const Status& s) {
  StaticJsonDocument<512> doc;

  // real_pose
  JsonObject real = doc.createNestedObject("real_pose");
  real["x"] = s.real_pose.x;
  real["y"] = s.real_pose.y;

  // target_pose
  JsonObject target = doc.createNestedObject("target_pose");
  target["x"] = s.target_pose.x;
  target["y"] = s.target_pose.y;

  // error
  JsonObject err = doc.createNestedObject("error");
  err["x"] = s.error.x;
  err["y"] = s.error.y;

  // platform_pose
  JsonObject pose = doc.createNestedObject("platform_pose");
  pose["x"] = s.platform_pose.x;
  pose["y"] = s.platform_pose.y;

  // pid_const
  JsonObject pid = doc.createNestedObject("pid_const");
  pid["kp_x"] = s.pid_const.kp_x;
  pid["ki_x"] = s.pid_const.ki_x;
  pid["kd_x"] = s.pid_const.kd_x;
  pid["kp_y"] = s.pid_const.kp_y;
  pid["ki_y"] = s.pid_const.ki_y;
  pid["kd_y"] = s.pid_const.kd_y;

  // joystick_val
  JsonObject joy = doc.createNestedObject("joystick_val");
  joy["x"] = s.joystick_val.x;
  joy["y"] = s.joystick_val.y;

  String output;
  serializeJson(doc, output);
  return output;
}

void platformInit() {
   String json = helloToJson(hello);
   client.publish(TOPIC_PUB_HELLO, json.c_str());
}

void mqtt_publish_status() {
   String json = statusToJson(status);
   client.publish(TOPIC_PUB_STATUS, json.c_str());
}

// ====== Wi-Fi 연결 ======
void wifiConnect()
{
   WiFi.mode(WIFI_STA);

   Serial.printf("Connecting to WiFi SSID: %s\n", ssid);
   WiFi.begin(ssid, password);

   // 선택: 초기에 연결 완료까지 대기하며 진행상태 출력
   unsigned long start = millis();
   while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
   {
      Serial.print('.');
      delay(500);
   }
   Serial.println();
   if (WiFi.status() == WL_CONNECTED)
   {
      Serial.print("Connected. IP: ");
      Serial.println(WiFi.localIP());
   }
   else
   {
      Serial.println("Initial connect timeout. Will keep trying via events.");
   }
}

// ====== MQTT 재접속 루프 ======
void mqttReconnect()
{
   while (!client.connected())
   {
      Serial.print("Attempting MQTT connection... ");
      // (옵션) LWT 설정: 끊길 때 상태 알림
      client.connect(UUID, nullptr, nullptr, "room/status", 0, false, "offline");
      if (client.connected())
      {
         Serial.print("connected as ");
         Serial.println(UUID);

         // 제어 토픽 구독
         client.subscribe(TOPIC_SUB_CMD, 0);
         Serial.print("Subscribed: ");
         Serial.println(TOPIC_SUB_CMD);
      }
      else
      {
         Serial.print("failed, rc=");
         Serial.print(client.state());
         Serial.println(" – retry in 3s");
         delay(3000);
      }
   }
}

// ====== MQTT 콜백 ======
void callback(char *topic, byte *payload, unsigned int length)
{
   Serial.print("Message arrived [");
   Serial.print(topic);
   Serial.print("] ");

   //메시지 파싱

   // 문자열 비교는 switch 대신 strcmp/if-else 사용
   if (strcmp(topic, TOPIC_SUB_CMD) == 0)
   {

   }
   else
   {
      // 다른 토픽은 무시 또는 로깅
      Serial.println("Unhandled topic");
   }
}


void setup(void)
{
   Serial.begin(115200);

   // 10 bit ADC values are expected by the touch library.
   // If touch locations seem incorrect, try uncommenting
   // this line to force 10 bit resolution:
   analogReadResolution(10);
   
   Serial.println();
   Serial.print("Connecting to ");
   Serial.println(ssid);

   wifiConnect();

   Serial.println("\nWiFi connected");
   Serial.print("IP address: ");
   Serial.println(WiFi.localIP());
   Serial.print("MAC: ");
   Serial.println(WiFi.macAddress());

   client.setServer(MQTT_HOST, MQTT_PORT);
   client.setCallback(callback);

   if (WiFi.status() != WL_CONNECTED)
   {
      wifiConnect();
   }
   if (!client.connected())
   {
      mqttReconnect();
   }

   platformInit();
}

void loop(void)
{
   static float timeCurr, timePrev;
   if (WiFi.status() != WL_CONNECTED)
   {
      wifiConnect();
   }
   if (!client.connected())
   {
      mqttReconnect();
   }
   client.loop();

   timeCurr = millis();
   float dt = (timeCurr - timePrev) / 1000; // get to seconds from milliseconds

   //  Serial.println(setpointX);
   if (dt > 0.02)
   {
      timePrev = timeCurr;
      // a point object holds x y and z coordinates
      TupleInt p = getPoint();
      Serial.print("X = ");
      Serial.print(p.x);
      Serial.print("\tY = ");
      Serial.println(p.y);
      status.real_pose.x = p.x;
      status.real_pose.y = p.y;
      mqtt_publish_status();
   }
}
