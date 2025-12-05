// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// This demo code returns raw readings, public domain

#include <Arduino.h>
#include <stdint.h>
#include <TouchScreen.h>

#define YP 2  // 3 must be an analog pin, use "An" notation!
#define XM 1  // 4 must be an analog pin, use "An" notation!
#define YM 41 // 1 can be a digital pin
#define XP 42 // 2 can be a digital pin

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 360);

struct Point
{
    int16_t x, y;
};

Point point_buffer[3];

Point getPoint()
{
    /*
     * p : 150 - 890 사이의 값에서 안정
     * actual_pose : x: -130~130, y:-100~100
     */
    point_buffer[0] = point_buffer[1];
    point_buffer[1] = point_buffer[2];
    Point p, actual_pose;
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

void setup(void)
{
    Serial.begin(115200);

    // 10 bit ADC values are expected by the touch library.
    // If touch locations seem incorrect, try uncommenting
    // this line to force 10 bit resolution:
    analogReadResolution(10);
}

void loop(void)
{
    // a point object holds x y and z coordinates
    Point p = getPoint();

    // we have some minimum pressure we consider 'valid'
    // pressure of 0 means no pressing!
    //   if (p.z > ts.pressureThreshhold) {
    //      Serial.print("X = "); Serial.print(p.x);
    //      Serial.print("\tY = "); Serial.print(p.y);
    //      Serial.print("\tPressure = "); Serial.println(p.z);
    //   }
    Serial.print("X = ");
    Serial.print(p.x);
    Serial.print("\tY = ");
    Serial.println(p.y);

    delay(100);
}
