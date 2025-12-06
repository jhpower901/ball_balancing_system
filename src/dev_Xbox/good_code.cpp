#include <Arduino.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "secrets.h"

#define UUID "esp32-balance-001"
#define VERSION "1.0.0"
#define TOPIC_PUB_HELLO "ballbalancer/hello"
#define TOPIC_PUB_STATUS "ballbalancer/status"
#define TOPIC_SUB_CMD "ballbalancer/cmd"

// ====== Wi-Fi & MQTT 설정 ======
const char *ssid      = WIFI_SSID;
const char *password  = WIFI_PASSWORD;
const char *MQTT_HOST = MQTT_HOSTNAME;
const uint16_t MQTT_PORT = MQTT_PORT_NUM;

// ====== 전역 객체 ======
WiFiClient espClient;
PubSubClient client(espClient);

// ====== Mutex (공유 변수 보호) ======
SemaphoreHandle_t pidMutex;
SemaphoreHandle_t statusMutex;

#define YP 2
#define XM 1
#define YM 41
#define XP 42

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 360);

// ====== SERVO 설정 ======
const int xServoPin = 19;
const int yServoPin = 20;
Servo xServo;
Servo yServo;
const int flatXAngle = 95;
const int flatYAngle = 90;

struct TupleInt {
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
   TupleFloat platform_pose= {0.0, 0.0}; // x: roll(output), y: pitch(output)
   PID_const pid_const = {
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0
   };
   TupleFloat joystick_val = {0.0, 0.0};
} status;

TupleInt point_buffer[3];

// ====== PID 제어 변수 ======
float Px = 0, Ix = 0, Dx = 0;
float Py = 0, Iy = 0, Dy = 0;
float previousErrorX = 0, previousErrorY = 0;

// ====== 입력 스무딩 ======
const int inputWindowSize = 10;
float filteredX = 0;
float filteredY = 0;
float sumX = 0;
float sumY = 0;
float readingsX[inputWindowSize];
float readingsY[inputWindowSize];

// ====== 유효성 체크 ======
int numValidPoints = 0;
int numInvalidPoints = 0;
int lastPressure = 0;


TupleInt getPoint()
{
   point_buffer[0] = point_buffer[1];
   point_buffer[1] = point_buffer[2];
   TupleInt p, actual_pose;
   TSPoint tp = ts.getPoint();
   
   lastPressure = tp.z;
   
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

int clip(int value, int minimum, int maximum) {
   if (value > maximum) return maximum;
   if (value < minimum) return minimum;
   return value;
}

float clip2(float value, float minimum, float maximum) {
   if (value > maximum) return maximum;
   if (value < minimum) return minimum;
   return value;
}

String helloToJson(const Hello& h) {
  StaticJsonDocument<512> doc;

  doc["device_id"] = h.device_id;
  doc["firmware"] = h.firmware;

  JsonObject pid = doc.createNestedObject("pid_const");
  
  if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
     pid["kp_x"] = h.pid_const.kp_x;
     pid["ki_x"] = h.pid_const.ki_x;
     pid["kd_x"] = h.pid_const.kd_x;
     pid["kp_y"] = h.pid_const.kp_y;
     pid["ki_y"] = h.pid_const.ki_y;
     pid["kd_y"] = h.pid_const.kd_y;
     xSemaphoreGive(pidMutex);
  }

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
  StaticJsonDocument<1024> doc; 

  if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
     JsonObject real = doc.createNestedObject("real_pose");
     real["x"] = s.real_pose.x;
     real["y"] = s.real_pose.y;

     JsonObject target = doc.createNestedObject("target_pose");
     target["x"] = s.target_pose.x;
     target["y"] = s.target_pose.y;

     JsonObject err = doc.createNestedObject("error");
     err["x"] = s.error.x;
     err["y"] = s.error.y;

     JsonObject pose = doc.createNestedObject("platform_pose");
     pose["roll"]  = (int)s.platform_pose.x; 
     pose["pitch"] = (int)s.platform_pose.y;

     JsonObject pid = doc.createNestedObject("pid_const");
     pid["kp_x"] = s.pid_const.kp_x;
     pid["ki_x"] = s.pid_const.ki_x;
     pid["kd_x"] = s.pid_const.kd_x;
     pid["kp_y"] = s.pid_const.kp_y;
     pid["ki_y"] = s.pid_const.ki_y;
     pid["kd_y"] = s.pid_const.kd_y;

     JsonObject joy = doc.createNestedObject("joystick_val");
     joy["x"] = s.joystick_val.x;
     joy["y"] = s.joystick_val.y;
     
     xSemaphoreGive(statusMutex);
  }

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

void wifiConnect()
{
   WiFi.mode(WIFI_STA);
   Serial.printf("[Core %d] Connecting to WiFi SSID: %s\n", xPortGetCoreID(), ssid);
   WiFi.begin(ssid, password);

   unsigned long start = millis();
   while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
   {
      Serial.print('.');
      delay(500);
   }
   Serial.println();
   if (WiFi.status() == WL_CONNECTED)
   {
      Serial.print("[Core ");
      Serial.print(xPortGetCoreID());
      Serial.print("] Connected. IP: ");
      Serial.println(WiFi.localIP());
   }
   else
   {
      Serial.println("[Core 0] Initial connect timeout. Will keep trying.");
   }
}

void mqttReconnect()
{
   static unsigned long lastAttempt = 0;
   
   if (millis() - lastAttempt > 3000)
   {
      lastAttempt = millis();
      
      if (!client.connected())
      {
         Serial.print("[Core 0] Attempting MQTT connection... ");
         
         if (client.connect(UUID, nullptr, nullptr, "room/status", 0, false, "offline"))
         {
            Serial.print("connected as ");
            Serial.println(UUID);
            client.subscribe(TOPIC_SUB_CMD, 0);
            Serial.print("Subscribed: ");
            Serial.println(TOPIC_SUB_CMD);
         }
         else
         {
            Serial.print("failed, rc=");
            Serial.println(client.state());
         }
      }
   }
}

void callback(char *topic, byte *payload, unsigned int length)
{
   Serial.print("\n[MQTT] Message arrived [");
   Serial.print(topic);
   Serial.print("] Length: ");
   Serial.println(length);

   if (strcmp(topic, TOPIC_SUB_CMD) == 0)
   {
      StaticJsonDocument<1024> doc;
      DeserializationError error = deserializeJson(doc, payload, length);
      
      if (error) {
         Serial.print("[Error] JSON parse failed: ");
         Serial.println(error.c_str());
         return;
      }

      // PID 상수 업데이트 (pid_const 내부 확인)
      if (doc.containsKey("pid_const")) 
      {
         JsonObject pid = doc["pid_const"];

         if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
            
            if (pid.containsKey("kp_x")) {
               float newVal = pid["kp_x"];
               hello.pid_const.kp_x = newVal;
               status.pid_const.kp_x = newVal;
               Serial.printf("[Update] kp_x changed to: %.2f\n", newVal);
            }
            if (pid.containsKey("ki_x")) {
               float newVal = pid["ki_x"];
               hello.pid_const.ki_x = newVal;
               status.pid_const.ki_x = newVal;
               Serial.printf("[Update] ki_x changed to: %.3f\n", newVal);
            }
            if (pid.containsKey("kd_x")) {
               float newVal = pid["kd_x"];
               hello.pid_const.kd_x = newVal;
               status.pid_const.kd_x = newVal;
               Serial.printf("[Update] kd_x changed to: %.3f\n", newVal);
            }
            if (pid.containsKey("kp_y")) {
               float newVal = pid["kp_y"];
               hello.pid_const.kp_y = newVal;
               status.pid_const.kp_y = newVal;
               Serial.printf("[Update] kp_y changed to: %.2f\n", newVal);
            }
            if (pid.containsKey("ki_y")) {
               float newVal = pid["ki_y"];
               hello.pid_const.ki_y = newVal;
               status.pid_const.ki_y = newVal;
               Serial.printf("[Update] ki_y changed to: %.3f\n", newVal);
            }
            if (pid.containsKey("kd_y")) {
               float newVal = pid["kd_y"];
               hello.pid_const.kd_y = newVal;
               status.pid_const.kd_y = newVal;
               Serial.printf("[Update] kd_y changed to: %.3f\n", newVal);
            }
            xSemaphoreGive(pidMutex);
         }
      }

      // 목표 위치(조이스틱) 업데이트
      if (doc.containsKey("target_pose")) 
      {
         JsonObject target = doc["target_pose"];
         
         if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
            if (target.containsKey("x")) {
               status.target_pose.x = target["x"];
               Serial.printf("[Update] Target X changed to: %d\n", status.target_pose.x);
            }
            if (target.containsKey("y")) {
               status.target_pose.y = target["y"];
               Serial.printf("[Update] Target Y changed to: %d\n", status.target_pose.y);
            }
            xSemaphoreGive(statusMutex);
         }
      }
      
      // 구버전 호환성 (joystick_x 직접 전송 시)
      if (doc.containsKey("joystick_x") || doc.containsKey("joystick_y")) {
          if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
             if (doc.containsKey("joystick_x")) {
                status.joystick_val.x = doc["joystick_x"];
                status.target_pose.x = (int16_t)(status.joystick_val.x * 130);
             }
             if (doc.containsKey("joystick_y")) {
                status.joystick_val.y = doc["joystick_y"];
                status.target_pose.y = (int16_t)(status.joystick_val.y * 100);
             }
             xSemaphoreGive(statusMutex);
          }
      }
   }
}

// ====================================================================
// Task Functions (setup보다 위에 있어야 함)
// ====================================================================

// Core 0 Task: MQTT 통신
void communicationTask(void *parameter)
{
   Serial.print("[Core 0] Communication task started on core ");
   Serial.println(xPortGetCoreID());
   
   while(1)
   {
      if (WiFi.status() != WL_CONNECTED) {
         static unsigned long lastWifiAttempt = 0;
         if (millis() - lastWifiAttempt > 10000) {
            lastWifiAttempt = millis();
            wifiConnect();
         }
      }
      
      mqttReconnect();
      
      if (client.connected()) {
         client.loop();
      }
      
      static unsigned long lastPublish = 0;
      if (millis() - lastPublish >= 20) 
      {
         lastPublish = millis();
         if (client.connected()) {
             mqtt_publish_status();
         }
      }
      vTaskDelay(1 / portTICK_PERIOD_MS);
   }
}

// Core 1 Task: PID 제어
void controlTask(void *parameter)
{
   Serial.print("[Core 1] Control task started on core ");
   Serial.println(xPortGetCoreID());
   
   float timeCurr, timePrev = 0;
   
   while(1)
   {
      timeCurr = millis();
      float dt = (timeCurr - timePrev) / 1000.0;
      
      if (dt > 0.02)
      {
         timePrev = timeCurr;
         TupleInt p = getPoint();
         
         if (lastPressure == 0) {
            numValidPoints = 0;
            numInvalidPoints++;
         } else {
            numValidPoints++;
            numInvalidPoints = 0;
         }

         // 리셋 조건 (터치 없음)
         if (numInvalidPoints >= 100)
         {
            xServo.write(flatXAngle);
            yServo.write(flatYAngle);
            Ix = 0; Iy = 0;
            
            if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
               status.platform_pose.x = 0; 
               status.platform_pose.y = 0; 
               status.real_pose.x = p.x;
               status.real_pose.y = p.y;
               xSemaphoreGive(statusMutex);
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
         }

         if (numValidPoints < 3) {
            if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
               status.real_pose.x = p.x;
               status.real_pose.y = p.y;
               xSemaphoreGive(statusMutex);
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
         }

         // ====== 유효 터치 시 (PID 제어 및 출력) ======
         if (lastPressure >= 10)
         {
            // 필터링
            sumX = sumX - readingsX[0];
            for (int i = 0; i < inputWindowSize - 1; i++) readingsX[i] = readingsX[i + 1];
            readingsX[inputWindowSize - 1] = p.x;
            sumX = sumX + p.x;
            filteredX = sumX / inputWindowSize;

            sumY = sumY - readingsY[0];
            for (int i = 0; i < inputWindowSize - 1; i++) readingsY[i] = readingsY[i + 1];
            readingsY[inputWindowSize - 1] = p.y;
            sumY = sumY + p.y;
            filteredY = sumY / inputWindowSize;

            float errorX, errorY;
            if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
               status.real_pose.x = (int16_t)filteredX;
               status.real_pose.y = (int16_t)filteredY;
               errorX = status.target_pose.x - filteredX;
               errorY = status.target_pose.y - filteredY;
               status.error.x = errorX;
               status.error.y = errorY;
               xSemaphoreGive(statusMutex);
            }

            float kp_x, ki_x, kd_x, kp_y, ki_y, kd_y;
            if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
               kp_x = hello.pid_const.kp_x; ki_x = hello.pid_const.ki_x; kd_x = hello.pid_const.kd_x;
               kp_y = hello.pid_const.kp_y; ki_y = hello.pid_const.ki_y; kd_y = hello.pid_const.kd_y;
               xSemaphoreGive(pidMutex);
            }

            Px = kp_x * errorX;
            Ix += ki_x * errorX * dt;
            Ix = clip2(Ix, -10, 10);
            Dx = kd_x * (errorX - previousErrorX) / dt;
            float PIDx = Px + Ix + Dx;

            Py = kp_y * errorY;
            Iy += ki_y * errorY * dt;
            Iy = clip2(Iy, -10, 10);
            Dy = kd_y * (errorY - previousErrorY) / dt;
            float PIDy = Py + Iy + Dy;

            int xOutput = int(round(map(PIDx, -130, 130, -50, 50)));
            int yOutput = int(round(map(PIDy, -100, 100, -40, 40)));
            
            xOutput = clip(xOutput, -50, 50);
            yOutput = clip(yOutput, -40, 40);

            // 서보 구동
            int actualXAngle = flatXAngle + xOutput;
            int actualYAngle = flatYAngle + yOutput;

            xServo.write(actualXAngle);
            yServo.write(actualYAngle);

            // MQTT Status 업데이트
            if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
               status.platform_pose.x = (float)xOutput; 
               status.platform_pose.y = (float)yOutput; 
               xSemaphoreGive(statusMutex);
            }

            previousErrorX = errorX;
            previousErrorY = errorY;

            // 시리얼 디버그 출력
            static unsigned long lastDebug = 0;
            if (millis() - lastDebug > 200) {
               lastDebug = millis();
               Serial.print("[Touch] Raw:(");
               Serial.print(p.x); Serial.print(","); Serial.print(p.y);
               Serial.print(") Filt:(");
               Serial.print(filteredX, 1); Serial.print(","); Serial.print(filteredY, 1);
               Serial.print(") Out:(");
               Serial.print(xOutput); Serial.print(","); Serial.print(yOutput);
               
               Serial.print(") | PID_X:(");
               Serial.print(kp_x, 2); Serial.print(",");
               Serial.print(ki_x, 3); Serial.print(",");
               Serial.print(kd_x, 2);
               Serial.print(") PID_Y:(");
               Serial.print(kp_y, 2); Serial.print(",");
               Serial.print(ki_y, 3); Serial.print(",");
               Serial.print(kd_y, 2);
               Serial.println(")");
            }
         }
         else {
             if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
               status.real_pose.x = p.x;
               status.real_pose.y = p.y;
               xSemaphoreGive(statusMutex);
            }
         }
      }
      vTaskDelay(1 / portTICK_PERIOD_MS);
   }
}

// ====================================================================
// SETUP & LOOP (반드시 가장 마지막에 위치)
// ====================================================================
void setup(void)
{
   Serial.begin(115200);
   analogReadResolution(10);
   
   Serial.println("\n========================================");
   Serial.println("ESP32 Ball Balancer Started");
   Serial.println("========================================");
   
   pidMutex = xSemaphoreCreateMutex();
   statusMutex = xSemaphoreCreateMutex();
   
   wifiConnect();

   client.setServer(MQTT_HOST, MQTT_PORT);
   client.setCallback(callback);
   client.setBufferSize(1024);

   if (WiFi.status() == WL_CONNECTED) mqttReconnect();

   xServo.attach(xServoPin);
   yServo.attach(yServoPin);
   xServo.write(flatXAngle);
   yServo.write(flatYAngle);

   status.pid_const = hello.pid_const;
   status.platform_pose.x = 0;
   status.platform_pose.y = 0;
   
   if (client.connected()) platformInit();
   

   //코어0, 코어1//
   xTaskCreatePinnedToCore(communicationTask, "Communication", 10000, NULL, 1, NULL, 0);
   xTaskCreatePinnedToCore(controlTask, "Control", 10000, NULL, 2, NULL, 1);
}

void loop(void)
{
   vTaskDelay(1000 / portTICK_PERIOD_MS);
}