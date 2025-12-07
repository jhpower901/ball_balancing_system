# ESP32 Ball Balancer Firmware

ESP32-S3 기반 볼 밸런서(Ball Balancer)를 구동하기 위한 **최종 펌웨어**입니다.  
터치스크린으로 공의 실제 위치를 측정하고, PID 제어로 두 개의 서보 모터를 제어하여 공을 목표 위치에 유지합니다.  

제어 입력은 다음 두 가지 모드를 지원합니다.

- **MQTT 모드** – Web Dashboard 서버에서 `target_pose` / PID 상수를 내려받아 제어  
- **Joystick 모드** – BLE Xbox 컨트롤러 조이스틱 및 버튼으로 직접 타깃 위치 제어  

MQTT를 통해 Web Dashboard와 양방향 통신하며, 실시간 상태를 JSON 형식으로 publish 합니다.

---

## ✨ 주요 기능

- **저지연 PID 제어 루프**
  - `CONTROL_LOOP_INTERVAL_MS = 10ms` (최대 100 Hz)
  - X/Y 축 각각 PID 상수(Kp, Ki, Kd) 제어
- **터치스크린 기반 공 위치 측정**
  - 노이즈/스파이크 필터링 (`SPIKE_THRESH`)
  - 라이브러리 레벨 스파이크 필터링 및 결측치 평균값 보간 (`NUMSAMPLES 20`)
  - 터치 센서 유효 출력 및 비선형성 보정을 위한 센서 값(150~890)을 실제 평면 좌표(약 −130~130, −100~100)로 맵핑
- **듀얼 서보 모터로 플랫폼 기울기 제어**
  - `X_SERVO_PIN`, `Y_SERVO_PIN`에 연결
  - 중립 각도(`flatXAngle`, `flatYAngle`) 기준 ±각도 출력
- **BLE Xbox 컨트롤러 입력**
  - Xbox버튼으로 joystick 모드 변경
  - 오른쪽 스틱으로 연속 타깃 이동
  - 버튼(X/Y/A/B, R Stick 버튼)으로 프리셋 타깃/센터 리셋
- **MQTT 연동**
  - Hello 패킷으로 초기 PID/필드 정보 전송
  - Status 패킷으로 실시간 상태 전송
  - CMD 패킷 수신으로 PID, 제어 모드, 타깃 위치 갱신
- **FreeRTOS 듀얼 코어 분리**
  - Core 0: Wi-Fi, MQTT, Xbox, 상태 publish (`communicationTask`)
  - Core 1: 터치 스캔 + PID 제어기 + 서보 구동 (`controlTask`)

---

## 🧱 하드웨어 구성

- **MCU**: ESP32-S3 (예: ESP32-S3 DevKitC-1)
- **센서**: 저항막 터치스크린 (XP/YP/XM/YM 4선식)
- **구동기**: 서보 모터 2개 (X, Y 축)
- **무선**: 2.4 GHz Wi-Fi, BLE (Xbox 컨트롤러)
- **기타**: MQTT 브로커가 동작 중인 서버 (예: ballbalancer Web Dashboard 서버)

핀 매핑(코드 기준):

```cpp
// TouchScreen
#define YP 2
#define XM 1
#define YM 41
#define XP 42

// Servo
#define X_SERVO_PIN 19
#define Y_SERVO_PIN 20
