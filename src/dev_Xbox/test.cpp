/*
 * Xbox Controller Example
 * -----------------------
 * This example demonstrates how to use the BLE-Gamepad-Client library to connect to an Xbox controller
 * https://github.com/tbekas/BLE-Gamepad-Client
 */

#include <Arduino.h>
#include <BLEGamepadClient.h>

XboxController controller;
bool wasConnected = false;
void setup() {
    Serial.begin(115200);
    controller.begin();
}
void loop() {
    if (controller.isConnected()) {
        XboxControlsEvent e;
        controller.read(&e);
        // Update connection status only if changed
        if (!wasConnected) {
            Serial.print("Connected");
            wasConnected = true;
        }
        // Update analog stick values
        Serial.printf("stick: L=%.2f / R=%.2f / ", e.leftStickX, e.leftStickY);
        Serial.printf("stick: L=%.2f / R=%.2f / ", e.rightStickX, e.rightStickY);
        // Triggers
        Serial.printf("trigger: L=%.2f / R=%.2f / ", e.leftTrigger, e.rightTrigger);
        // Face buttons
        if (e.buttonA)
            Serial.print("A ");
        if (e.buttonB)
            Serial.print("B ");
        if (e.buttonX)
            Serial.print("X ");
        if (e.buttonY)
            Serial.print("Y ");
        // Bumpers and stick buttons
        if (e.leftBumper)
            Serial.print("LB ");
        if (e.rightBumper)
            Serial.print("RB ");
        if (e.leftStickButton)
            Serial.print("LSB ");
        if (e.rightStickButton)
            Serial.print("RSB ");
        // D-pad
        if (e.dpadUp)
            Serial.print("Up ");
        if (e.dpadDown)
            Serial.print("Down ");
        if (e.dpadLeft)
            Serial.print("Left ");
        if (e.dpadRight)
            Serial.print("Right ");
        // Center buttons
        if (e.shareButton)
            Serial.print("Share ");
        if (e.menuButton)
            Serial.print("Menu ");
        if (e.viewButton)
            Serial.print("View ");
        if (e.xboxButton)
            Serial.print("Xbox ");
        Serial.println();
    } else {
        if (wasConnected) {
            Serial.print("Controller not connected");
            wasConnected = false;
        }
    }
    delay(100);
}