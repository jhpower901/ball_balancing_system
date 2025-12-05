#include <Arduino.h>

// 핀 번호 정의 (보드마다 LED 핀 다를 수 있음)
// ESP32-S3 DevKitC-1 기본 LED는 대부분 GPIO 48
#define LED_PIN 2

void setup() {
    // 핀을 OUTPUT(출력) 모드로 설정
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // LED 켜기 (HIGH = 1)
    digitalWrite(LED_PIN, HIGH);
    delay(100);     // 500ms(0.5초) 기다리기

    // LED 끄기 (LOW = 0)
    digitalWrite(LED_PIN, LOW);
    delay(100);     // 500ms 기다리기
}
