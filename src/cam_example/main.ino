#include <WiFi.h>

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

#ifdef PIN_LED_RGB
#define BUILTIN_RGBLED_PIN PIN_LED_RGB
#else
#define BUILTIN_RGBLED_PIN 48  // 보드에 맞게 조정 (일반적으로 48, 38, 8 등)
#endif

#define NR_OF_LEDS 1  // 단일 RGB LED
#define NR_OF_ALL_BITS 24 * NR_OF_LEDS

rmt_data_t led_data[NR_OF_ALL_BITS];

WiFiServer server(80);

void setup() {
    Serial.begin(115200);
    
    // RMT 초기화
    if (!rmtInit(BUILTIN_RGBLED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) {
        Serial.println("RMT init failed");
    }
    Serial.println("RMT tick set to: 100ns");
    
    delay(10);
    
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    
    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address:");
    Serial.println(WiFi.localIP());
    server.begin();
}

void setLEDColor(int red, int green, int blue) {
    int color[] = {green, red, blue};  // WS2812는 GRB 순서
    int i = 0;
    
    for (int led = 0; led < NR_OF_LEDS; led++) {
        for (int col = 0; col < 3; col++) {
            for (int bit = 0; bit < 8; bit++) {
                if (color[col] & (1 << (7 - bit))) {
                    // Bit 1: 0.8us HIGH, 0.4us LOW
                    led_data[i].level0 = 1;
                    led_data[i].duration0 = 8;
                    led_data[i].level1 = 0;
                    led_data[i].duration1 = 4;
                } else {
                    // Bit 0: 0.4us HIGH, 0.85us LOW
                    led_data[i].level0 = 1;
                    led_data[i].duration0 = 4;
                    led_data[i].level1 = 0;
                    led_data[i].duration1 = 8;
                }
                i++;
            }
        }
    }
    
    // RMT로 데이터 전송
    rmtWrite(BUILTIN_RGBLED_PIN, led_data, NR_OF_ALL_BITS, RMT_WAIT_FOR_EVER);
}

void loop() {
    WiFiClient client = server.available();
    
    if(client) {
        Serial.println("New Client.");
        String currentLine = "";
        
        while(client.connected()) {
            if(client.available()) {
                char c = client.read();
                Serial.write(c);
                
                if(c == '\n') {
                    if(currentLine.length() == 0) {
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println();
                        client.print("<font size='100'>RGB LED Control (RMT)<br>");
                        client.print("Click <a href=\"/RED\">here</a> RED ON<br>");
                        client.print("Click <a href=\"/GREEN\">here</a> GREEN ON<br>");
                        client.print("Click <a href=\"/BLUE\">here</a> BLUE ON<br>");
                        client.print("Click <a href=\"/YELLOW\">here</a> YELLOW ON<br>");
                        client.print("Click <a href=\"/PURPLE\">here</a> PURPLE ON<br>");
                        client.print("Click <a href=\"/CYAN\">here</a> CYAN ON<br>");
                        client.print("Click <a href=\"/WHITE\">here</a> WHITE ON<br>");
                        client.print("Click <a href=\"/OFF\">here</a> LED OFF<br>");
                        client.println();
                        break;
                    } else {
                        currentLine = "";
                    }
                } else if (c != '\r') {
                    currentLine += c;
                }
                
                // RGB LED 제어 (RMT 방식)
                if (currentLine.endsWith("GET /RED")) {
                    setLEDColor(255, 0, 0);
                }
                if (currentLine.endsWith("GET /GREEN")) {
                    setLEDColor(0, 255, 0);
                }
                if (currentLine.endsWith("GET /BLUE")) {
                    setLEDColor(0, 0, 255);
                }
                if (currentLine.endsWith("GET /YELLOW")) {
                    setLEDColor(255, 255, 0);
                }
                if (currentLine.endsWith("GET /PURPLE")) {
                    setLEDColor(255, 0, 255);
                }
                if (currentLine.endsWith("GET /CYAN")) {
                    setLEDColor(0, 255, 255);
                }
                if (currentLine.endsWith("GET /WHITE")) {
                    setLEDColor(255, 255, 255);
                }
                if (currentLine.endsWith("GET /OFF")) {
                    setLEDColor(0, 0, 0);
                }
            }
        }
        
        client.stop();
        Serial.println("Client Disconnected.");
    }
}
