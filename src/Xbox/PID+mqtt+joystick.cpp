#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <BLEGamepadClient.h>


#define UUID "esp32-balance-001"
#define VERSION "1.0.0"
#define TOPIC_PUB_HELLO "ballbalancer/hello"
#define TOPIC_PUB_STATUS "ballbalancer/status"
#define TOPIC_SUB_CMD "ballbalancer/cmd"

#define SPIKE_THRESH 20      // 터치포인트 급변 필터링 임계값
#define STEP_GAIN 3.0f                 // 조이스틱 1.0일 때 호출 한 번당 타깃 이동 거리
#define STATUS_PUBLISH_INTERVAL_MS 20  // 상태 MQTT 발행 주기
#define CONTROL_LOOP_INTERVAL_MS 10.0  // 제어 루프 주기

// ====== PID constants initial values ======
#define KP_X 0.38f
#define KI_X 0.05f
#define KD_X 0.12f
#define KP_Y 0.28f
#define KI_Y 0.03f
#define KD_Y 0.12f

// ====== Wi-Fi & MQTT 설정 ======
const char *ssid = "jsj";
const char *password = "19960828";
const char *MQTT_HOST = "anzam.kr";
const uint16_t MQTT_PORT = 1883;

// ====== 전역 객체 ======
WiFiClient espClient;
PubSubClient client(espClient);
XboxController controller;

// ====== Mutex (공유 변수 보호) ======
SemaphoreHandle_t pidMutex;
SemaphoreHandle_t statusMutex;

// ====== 터치스크린 설정 ======
#define YP 2
#define XM 1
#define YM 41
#define XP 42
#define ZERO_POINT_CALIBRATION_X 0
#define ZERO_POINT_CALIBRATION_Y 0

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 360);

// ====== SERVO 설정 ======
#define X_SERVO_PIN 19
#define Y_SERVO_PIN 20
Servo xServo;
Servo yServo;
const int flatXAngle = 95;    // 서보 중립각(초기 설정 calibration중 조정 가능)
const int flatYAngle = 95;    // 서보 중립각(초기 설정 calibration중 조정 가능)

// ===== Xbox 컨트롤러 연결 상태 ======
bool wasConnected = false;

// ===== 데이터 구조체 ======
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

enum ControlMode : uint8_t {
   CTR_MODE_JOYSTICK = 0,  // Xbox 조이스틱 모드
   CTR_MODE_MQTT = 1       // MQTT에서 target_pose 직접 지정
};

volatile ControlMode g_controlMode = CTR_MODE_MQTT;

// ====== MQTT 전송용 데이터 ======
struct Hello {
   char device_id[100] = UUID;
   char firmware[10] = VERSION;
   struct PID_const pid_const = {
      KP_X, KI_X, KD_X,
      KP_Y, KI_Y, KD_Y
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
   uint32_t time = 0;
   char ctr_mode[20] = "joystick";
} status;

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


// ====================================================================
// HELPER FUNCTIONS
// ====================================================================
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

// ====== 터치포인트 좌표 반환 ======
TupleInt point_buffer[3];
TupleInt getPoint() {
   /*
    * p : 150 - 890 사이의 값에서 안정
    * actual_pose : x: -130~130, y:-100~100
    */
   point_buffer[0] = point_buffer[1];
   point_buffer[1] = point_buffer[2];
   TupleInt p, actual_pose;
   TSPoint tp = ts.getPoint();
   
   lastPressure = tp.z;
   int abs;

   // 유효한 터치 포인트인지 검사 및 보간 처리
   if (tp.z > 0) {
      if ((abs = tp.x - point_buffer[1].x > 0 ? abs : -abs) > SPIKE_THRESH)
         p.x = point_buffer[1].x + (point_buffer[1].x - point_buffer[0].x);
      else
         p.x = tp.x;
      if ((abs = tp.y - point_buffer[1].y > 0 ? abs : -abs) > SPIKE_THRESH)
         p.y = point_buffer[1].y + (point_buffer[1].y - point_buffer[0].y);
      else
         p.y = tp.y;
   } else {
      p.x = point_buffer[1].x + (point_buffer[1].x - point_buffer[0].x);
      p.y = point_buffer[1].y + (point_buffer[1].y - point_buffer[0].y);
   }

   // 유효값 클리핑. 범위 넘어갈 시 이전 값 유지
   if (p.x < 0 || p.x > 1023) {
      p.x = point_buffer[1].x;
   }
   if (p.y < 0 || p.y > 1023) {
      p.y = point_buffer[1].y;
   }

   point_buffer[2] = p;
   actual_pose.x = map(clip(p.x + ZERO_POINT_CALIBRATION_X, 150, 890), 150, 890, -130, 130);
   actual_pose.y = map(clip(p.y + ZERO_POINT_CALIBRATION_Y, 150, 890), 150, 890, -100, 100);
   return actual_pose;
}


// ====== MQTT JSON 변환 함수 ======
String helloToJson(const Hello& h) {
  JsonDocument doc;

  doc["device_id"] = h.device_id;
  doc["firmware"]  = h.firmware;

  // pid_const
  JsonObject pid = doc["pid_const"].to<JsonObject>();

  if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
     pid["kp_x"] = h.pid_const.kp_x;
     pid["ki_x"] = h.pid_const.ki_x;
     pid["kd_x"] = h.pid_const.kd_x;
     pid["kp_y"] = h.pid_const.kp_y;
     pid["ki_y"] = h.pid_const.ki_y;
     pid["kd_y"] = h.pid_const.kd_y;
     xSemaphoreGive(pidMutex);
  }

  // platform_pose
  JsonObject pose = doc["platform_pose"].to<JsonObject>();
  pose["roll"]  = h.platform_pose.roll;
  pose["pitch"] = h.platform_pose.pitch;

  // field_size
  JsonObject fs = doc["field_size"].to<JsonObject>();
  fs["width"]  = h.field_size.width;
  fs["height"] = h.field_size.height;

  String output;
  serializeJson(doc, output);
  return output;
}

String statusToJson(const Status& s) {
  JsonDocument doc;

  if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
     JsonObject real = doc["real_pose"].to<JsonObject>();
     real["x"] = s.real_pose.x;
     real["y"] = s.real_pose.y;

     JsonObject target = doc["target_pose"].to<JsonObject>();
     target["x"] = s.target_pose.x;
     target["y"] = s.target_pose.y;

     JsonObject err = doc["error"].to<JsonObject>();
     err["x"] = s.error.x;
     err["y"] = s.error.y;

     JsonObject pose = doc["platform_pose"].to<JsonObject>();
     pose["roll"]  = (int)s.platform_pose.x;
     pose["pitch"] = (int)s.platform_pose.y;

     JsonObject pid = doc["pid_const"].to<JsonObject>();
     pid["kp_x"] = s.pid_const.kp_x;
     pid["ki_x"] = s.pid_const.ki_x;
     pid["kd_x"] = s.pid_const.kd_x;
     pid["kp_y"] = s.pid_const.kp_y;
     pid["ki_y"] = s.pid_const.ki_y;
     pid["kd_y"] = s.pid_const.kd_y;

     JsonObject joy = doc["joystick_val"].to<JsonObject>();
     joy["x"] = s.joystick_val.x;
     joy["y"] = s.joystick_val.y;

     doc["time"]     = s.time;
     doc["ctr_mode"] = s.ctr_mode;

     xSemaphoreGive(statusMutex);
  }

  String output;
  serializeJson(doc, output);
  return output;
}

// ====== MQTT 메시지 발행 함수 ======
void mqtt_publish_status() {
   String json = statusToJson(status);
   client.publish(TOPIC_PUB_STATUS, json.c_str());
}

// ====== 부팅 시 initializing ======
void platformInit() {
   String json = helloToJson(hello);
   client.publish(TOPIC_PUB_HELLO, json.c_str());
}

// ====== MQTT 수신 메시지 파싱 및 상태 업데이트 ======
void updateStatusFromJson(const JsonDocument& doc) {
   if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {

      // --- time ---
      if (doc["time"].is<uint32_t>()) {
         status.time = doc["time"].as<uint32_t>();
         Serial.printf("[Update] time = %d\n", status.time);
      }

      // --- ctr_mode (문자열 → enum + status.ctr_mode 동기화) ---
      if (doc["ctr_mode"].is<const char*>()) {
         const char* modeStr = doc["ctr_mode"].as<const char*>();
         if (modeStr != nullptr) {
            // enum 값 결정
            if (strcmp(modeStr, "joystick") == 0) {
               g_controlMode = CTR_MODE_JOYSTICK;
            } else if (strcmp(modeStr, "mqtt") == 0) {
               g_controlMode = CTR_MODE_MQTT;
            }
            // 문자열 상태에도 저장 (MQTT status용)
            strlcpy(status.ctr_mode, modeStr, sizeof(status.ctr_mode));
            Serial.printf("[Mode] ctr_mode set by MQTT: %s\n", status.ctr_mode);
         }
      }

      // --- target_pose (MQTT에서 직접 주는 경우) ---
      JsonObjectConst target = doc["target_pose"];
      if (!target.isNull()) {
         // x
         if (!target["x"].isNull()) {
            status.target_pose.x = target["x"].as<int>();   // 내부 타입이 float이든 long이든 걍 int로 변환
         }
         // y
         if (!target["y"].isNull()) {
            status.target_pose.y = target["y"].as<int>();
         }

         Serial.printf("[Update] target X = %d Y = %d\n",
                     status.target_pose.x, status.target_pose.y);
      }
      xSemaphoreGive(statusMutex);
   }
}


// ====== 조이스틱 입력으로 상태 업데이트 ======
void updateStatusFromJoystick(XboxControlsEvent e) {
   // 한 번에 움직이는 양 (튜닝 포인트)
   const float MAX_TARGET_X  = 130.0f; // 센서 맵핑 범위와 맞춤
   const float MAX_TARGET_Y  = 100.0f;

   if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {

      // 1) Xbox 버튼 눌리면 제어 모드 전환
      if (e.xboxButton) {
         strlcpy(status.ctr_mode, "joystick", sizeof(status.ctr_mode));
         Serial.println("[Joystick] Control mode = joystick (Xbox button)");
      }

      // 2) 현재 조이스틱 값 기록 (모니터링 / MQTT용)
      status.joystick_val.x = e.rightStickX;
      status.joystick_val.y = e.rightStickY;

      // 3) 오른쪽 스틱 버튼 누르면 타깃 원점으로 리셋
      if (e.rightStickButton) {
         status.target_pose.x = 0;
         status.target_pose.y = 0;
         Serial.println("[Joystick] Target pose reset to (0, 0) by RSB");
      }

      // 4) 모드가 joystick일 때만 타깃을 조이스틱으로 이동
      if (strcmp(status.ctr_mode, "joystick") == 0) {
         float deltaX = e.rightStickX * STEP_GAIN;
         float deltaY = e.rightStickY * STEP_GAIN;

         float newX = status.target_pose.x + deltaX;
         float newY = status.target_pose.y + deltaY;

         // 범위 클리핑
         clip2(newX, -MAX_TARGET_X, MAX_TARGET_X);
         clip2(newY, -MAX_TARGET_Y, MAX_TARGET_Y);

         status.target_pose.x = (int16_t)newX;
         status.target_pose.y = (int16_t)newY;
      }

      xSemaphoreGive(statusMutex);
   }
}


// ====== Wi-Fi 및 MQTT 연결 함수 ======
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
         
         if (client.connect(UUID, nullptr, nullptr, "ballbalancer/hello", 0, false, "offline"))
         {
            Serial.print("connected as ");
            Serial.println(UUID);
            client.subscribe(TOPIC_SUB_CMD, 0);
            Serial.print("Subscribed: ");
            Serial.println(TOPIC_SUB_CMD);
            platformInit();
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
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload, length);
      
      if (error) {
         Serial.print("[Error] JSON parse failed: ");
         Serial.println(error.c_str());
         return;
      }

      // 1) PID 상수 업데이트
      if (doc["pid_const"].is<JsonObject>()) 
      {
         JsonObject pid = doc["pid_const"].as<JsonObject>();

         if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
            if (pid["kp_x"].is<float>()) {
               float v = pid["kp_x"].as<float>();
               status.pid_const.kp_x = v;
               Serial.printf("[Update] kp_x = %.3f\n", v);
            }
            if (pid["ki_x"].is<float>()) {
               float v = pid["ki_x"].as<float>();
               status.pid_const.ki_x = v;
               Serial.printf("[Update] ki_x = %.3f\n", v);
            }
            if (pid["kd_x"].is<float>()) {
               float v = pid["kd_x"].as<float>();
               status.pid_const.kd_x = v;
               Serial.printf("[Update] kd_x = %.3f\n", v);
            }
            if (pid["kp_y"].is<float>()) {
               float v = pid["kp_y"].as<float>();
               status.pid_const.kp_y = v;
               Serial.printf("[Update] kp_y = %.3f\n", v);
            }
            if (pid["ki_y"].is<float>()) {
               float v = pid["ki_y"].as<float>();
               status.pid_const.ki_y = v;
               Serial.printf("[Update] ki_y = %.3f\n", v);
            }
            if (pid["kd_y"].is<float>()) {
               float v = pid["kd_y"].as<float>();
               status.pid_const.kd_y = v;
               Serial.printf("[Update] kd_y = %.3f\n", v);
            }
            xSemaphoreGive(pidMutex);
         }
      }

      // 2) Status 관련 필드(time, ctr_mode, target_pose, joystick 등) 업데이트
      updateStatusFromJson(doc);
   }
}


// ====================================================================
// Task Functions
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
      
      // 조이스틱 읽기
      static unsigned long lastJoystickRead = 0;
      if (millis() - lastJoystickRead >= 100) {
         lastJoystickRead = millis();
         if (controller.isConnected()) {
         XboxControlsEvent e;
         controller.read(&e);
         // Update connection status only if changed
         if (!wasConnected) {
               Serial.print("Connected");
               wasConnected = true;
         }
         updateStatusFromJoystick(e);
         } else {
            if (wasConnected) {
               Serial.print("Controller not connected");
               wasConnected = false;
            }
         }
      }

      // 주기적 상태 발행 
      static unsigned long lastPublish = 0;
      if (millis() - lastPublish >= STATUS_PUBLISH_INTERVAL_MS) {
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
      
      if (dt > CONTROL_LOOP_INTERVAL_MS / 1000.0)
      {
         timePrev = timeCurr;
         TupleInt p = getPoint();
         //Serial.printf("X=%d, Y=%d\n", p.x, p.y);


         // ====== 터치 입력 검사 >> 터치 없을 시 플랫폼 원점 복귀 ======
         if (lastPressure <= 0) {
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

         float errorX, errorY;
         int targetX, targetY;
         if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
            status.real_pose.x = (int16_t)p.x;
            status.real_pose.y = (int16_t)p.y;
            targetX = status.target_pose.x;
            targetY = status.target_pose.y;
            errorX = (float)targetX - p.x;
            errorY = (float)targetY - p.y;
            status.error.x = errorX;
            status.error.y = errorY;
            xSemaphoreGive(statusMutex);
         }

         float kp_x, ki_x, kd_x, kp_y, ki_y, kd_y;
         if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE) {
            kp_x = status.pid_const.kp_x;
            ki_x = status.pid_const.ki_x;
            kd_x = status.pid_const.kd_x;
            kp_y = status.pid_const.kp_y;
            ki_y = status.pid_const.ki_y;
            kd_y = status.pid_const.kd_y;
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
         if (millis() - lastDebug > 1000) {
            lastDebug = millis();
            Serial.print("[Touch] Raw:(");
            Serial.print(p.x); Serial.print(","); Serial.print(p.y);
            Serial.print(") Target:(");
            Serial.print(targetX); Serial.print(","); Serial.print(targetY);
            Serial.print(") Err:(");
            Serial.print(errorX); Serial.print(","); Serial.print(errorY);
            Serial.print(") Out:(");
            Serial.print(xOutput); Serial.print(","); Serial.print(yOutput);
            Serial.printf(") [Joystick]: %s", wasConnected ? "Connected" : "Disconnected");
            
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
      vTaskDelay(1 / portTICK_PERIOD_MS);
   }
}

// ====================================================================
// SETUP & LOOP
// ====================================================================
void setup(void)
{
   Serial.begin(115200);
   controller.begin();
   analogReadResolution(10);     // TouchScreen 해상도 설정
   
   Serial.println("\n========================================");
   Serial.println("ESP32 Ball Balancer Started");
   Serial.println("========================================");
   
   pidMutex = xSemaphoreCreateMutex();
   statusMutex = xSemaphoreCreateMutex();
   
   wifiConnect();

   client.setServer(MQTT_HOST, MQTT_PORT);
   client.setCallback(callback);
   client.setBufferSize(1024);

   if (WiFi.status() == WL_CONNECTED)
      mqttReconnect();

   xServo.attach(X_SERVO_PIN);
   yServo.attach(Y_SERVO_PIN);
   xServo.write(flatXAngle);
   yServo.write(flatYAngle);

   status.pid_const = hello.pid_const;
   status.platform_pose.x = 0;
   status.platform_pose.y = 0;
   
   //코어0, 코어1//
   xTaskCreatePinnedToCore(communicationTask, "Communication", 10000, NULL, 1, NULL, 0);
   xTaskCreatePinnedToCore(controlTask, "Control", 10000, NULL, 2, NULL, 1);
}

void loop(void)
{
   vTaskDelay(1000 / portTICK_PERIOD_MS);
}